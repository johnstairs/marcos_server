#include "hardware.hpp"
#include "iface.hpp"
#include <fcntl.h>
#include <cmath>
#include <cassert>
#include <chrono>
#include <ctime>
#include <vector>
#include <sys/mman.h>

#ifdef VERILATOR_BUILD
#include "marga_model.hpp"
extern marga_model *mm;
#endif

// variadic macro for debugging enable/disable
// #define debug_printf(...) printf(__VA_ARGS__)
#define debug_printf(...)

hardware::hardware() {
	init_mem();
}

hardware::~hardware() {
}

int hardware::run_request(server_action &sa) {
	// See whether HW needs to be reconfigured
	size_t commands_present = sa.command_count();
	size_t commands_understood = 0;
	auto wr = sa.get_writer();
	int problems = 0;
	int status;
	mpack_start_map(wr, commands_present); // each command should fill in an element of the map

	if (commands_present == 0) {
		++problems;
		sa.add_error("no commands present or incorrectly formatted request");
	}

	// Halt and reset; returns true if the FSM has already halted (may take up to 2ms to halt the HDL if it's counting down
	sa.get_command_and_start_reply("halt_and_reset", status);
	if (status == 1) {
		++commands_understood;
		halt_and_reset(&sa);

		auto exec = rd32(_exec);
		bool halted = (exec >> 24) == MAR_STATE_IDLE;
		mpack_write(wr, halted);
	}

	// Read directly from memory [TODO: implement]
	sa.get_command_and_start_reply("read_mem", status);
	if (status == 1) {
		++commands_understood;
		mpack_write(wr, c_ok);
	}

	// FPGA clock config [TODO: understand this better]
	auto fcwa1 = sa.get_command_and_start_reply("fpga_clk", status);
	if (status == 1) {
		++commands_understood;
		// Enforce that all three words are present for FPGA clock configuration
		if (mpack_node_array_length(fcwa1) != 3) {
			sa.add_error("you only provided some FPGA clock control words; check you're providing all 3");
			mpack_write(wr, c_err); // error
		} else {
			_slcr[2] = mpack_node_u32(mpack_node_array_at(fcwa1, 0));
			_slcr[92] = (_slcr[92] & ~mpack_node_u32(mpack_node_array_at(fcwa1, 1)))
			        | mpack_node_u32(mpack_node_array_at(fcwa1, 2));
			mpack_write(wr, c_ok); // okay
		}
	} else if (status == -1) {
		// sa.add_error("Unknown MPack error from fpga_clk");
		// TODO: callback or similar
	}

	// Alter main control register
	auto ctrl = sa.get_command_and_start_reply("ctrl", status);
	if (status == 1) {
		++commands_understood;
		wr32(_ctrl, mpack_node_u32(ctrl));
		mpack_write(wr, c_ok);
	}

	// Command directly to the buffers
	auto dir = sa.get_command_and_start_reply("direct", status);
	if (status == 1) {
		++commands_understood;
		wr32(_direct, mpack_node_u32(dir));
		// TODO: add sanity check for instruction/data type
		// TODO: implement higher-level direct commands, like 32b writes etc
		mpack_write(wr, c_ok);
	}

	// Register the per-channel "zero current" direct-write words used by
	// halt_and_reset() to park the gradient DACs at midpoint after an
	// emergency stop. The encoding is board-specific, so the client
	// computes the words (via grad_board.float2bin + marcompile.col2buf)
	// and hands them over here rather than the server hard-coding them.
	//
	// _gpa_zero_words is process-lifetime state on the singleton hardware
	// instance: once a client registers the words, they persist across
	// client disconnects until the marcos_server process exits or another
	// set_gpa_zero_words call overwrites them. Monarch relies on this --
	// the initial InitGpas action (or any earlier Experiment with
	// init_gpa=True) registers the words, and subsequent Experiments that
	// pass init_gpa=False inherit them without re-registering.
	auto gzw = sa.get_command_and_start_reply("set_gpa_zero_words", status);
	if (status == 1) {
		++commands_understood;
		// Both supported gradient boards (OCRA1 and GPA-FHDO) expose 4
		// channels, so the client must send exactly one zero-word per
		// channel. A mismatch means the client is out of sync with the
		// board configuration; reject the update rather than partially
		// overwriting the previously-registered vector.
		static constexpr size_t expected_gpa_zero_words = 4;
		size_t n = mpack_node_array_length(gzw);
		if (n != expected_gpa_zero_words) {
			char t[160];
			sprintf(t, "set_gpa_zero_words: received %zu words, expected %zu"
			        " (one per gradient channel); keeping previously-registered"
			        " words unchanged.", n, expected_gpa_zero_words);
			sa.add_warning(t);
			mpack_write(wr, c_warn);
		} else {
			_gpa_zero_words.clear();
			_gpa_zero_words.reserve(n);
			for (size_t i = 0; i < n; ++i) {
				_gpa_zero_words.push_back(mpack_node_u32(mpack_node_array_at(gzw, i)));
			}
			mpack_write(wr, c_ok);
		}
	}

	// Read one register
	auto regidx = sa.get_command_and_start_reply("regrd", status);
	if (status == 1) {
		++commands_understood;
		mpack_write(wr, rd32(_mar_base + mpack_node_u32(regidx)));
	}

	// Read all registers
	sa.get_command_and_start_reply("regstatus", status);
	if (status == 1) {
		++commands_understood;
		mpack_start_array(wr, 7);

		mpack_write(wr, rd32(_exec));
		mpack_write(wr, rd32(_status));
		mpack_write(wr, rd32(_status_latch));
		mpack_write(wr, rd32(_buf_err));
		mpack_write(wr, rd32(_buf_full));
		mpack_write(wr, rd32(_buf_empty));
		mpack_write(wr, rd32(_rx_locs));

		mpack_finish_array(wr);
	}

	// Fill in marga execution memory
	// TODO: add an input offset too, to avoid having to overwrite everything every time
	auto mm = sa.get_command_and_start_reply("mar_mem", status);
	if (status == 1) {
		++commands_understood;
		char t[100];

		// uint32_t ro = *(uint32_t *)(GRAD_CTRL_REG_OFFSET);
		if (mpack_node_bin_size(mm) <= MARGA_MEM_SIZE) {
			size_t bytes_copied = hw_mpack_node_copy_data(mm, _mar_mem, MARGA_MEM_SIZE);
			sprintf(t, "mar mem data bytes copied: %zu", bytes_copied);
			sa.add_info(t);
			mpack_write(wr, c_ok);
		} else {
			sprintf(t, "too much mar mem data: %zu bytes > %d -- streaming not yet implemented", mpack_node_bin_size(mm), MARGA_MEM_SIZE);
			sa.add_error(t);
			mpack_write(wr, c_err);
		}
	}

	// Configure acquisition retry limit (how many times the server will poll the RX FIFO when waiting for data)
	auto arl = sa.get_command_and_start_reply("acq_rlim", status);
	if (status == 1) {
		++commands_understood;
		uint32_t rl = mpack_node_u32(arl);
		if (rl < 1000 or rl > 10000000) {
			sa.add_error("acquisition retry limit outside the range [1000, 10,000,000]; check your settings");
			mpack_write(wr, c_err);
		} else {
			_read_tries_limit = rl;
			mpack_write(wr, c_ok);
		}
	}

	// read all outstanding data from RX FIFOs
	sa.get_command_and_start_reply("read_rx", status);
	if (status == 1) {
		++commands_understood;
		char t[100];
		std::vector<uint32_t> rx0_i, rx0_q, rx1_i, rx1_q;
		read_rx(rx0_i, rx0_q, rx1_i, rx1_q);

		// encode the RX replies
		unsigned rx0_elem = rx0_i.size(), rx1_elem = rx1_i.size();
		if (!rx0_elem and !rx1_elem) {
			mpack_write(wr, c_ok);
			sprintf(t, "no RX data received");
			sa.add_warning(t);
		} else {
			mpack_start_map(wr, (rx0_elem ? 2 : 0) + (rx1_elem ? 2 : 0));
			if (rx0_elem) {
				mpack_write_cstr(wr, "rx0_i");
				mpack_start_array(wr, rx0_elem);
				for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx0_q");
				mpack_start_array(wr, rx0_elem);
				for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_q[k]);
				mpack_finish_array(wr);
			}

			if (rx1_elem) {
				mpack_write_cstr(wr, "rx1_i");
				mpack_start_array(wr, rx1_elem);
				for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx1_q");
				mpack_start_array(wr, rx1_elem);
				for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_q[k]);
				mpack_finish_array(wr);
			}

			mpack_finish_map(wr);
		}
	}

	// Run a sequence
	auto runs = sa.get_command_and_start_reply("run_seq", status);
	if (status == 1) {
		++commands_understood;
		char t[100];

		// ensure that nothing is currently running; halt if it is
		auto exec = rd32(_exec);
		if ((exec >> 24) != MAR_STATE_IDLE) {
			sprintf(t, "mar FSM was not idle when the run began");
			sa.add_warning(t);
			halt();
		}

		// Discard any stale RX samples left from a prior run
		// (e.g. after abort or client disconnect).  FSM is stopped
		// so no new data is arriving — a small idle count suffices.
		discard_rx();

		const size_t total_bytes_to_copy = mpack_node_bin_size(runs);
		debug_printf("total bytes to copy: %zu\n", total_bytes_to_copy);
		const char *rundata = (char *)mpack_node_bin_data(runs);
		size_t mem_offset = 0;

		// initially fill the marga memory
		if (total_bytes_to_copy <= MARGA_MEM_SIZE) {
			hw_memcpy(_mar_mem, rundata, total_bytes_to_copy);
			mem_offset += total_bytes_to_copy;
		} else {
			// first time, wrap around back to the start again,
			// so don't need to update marga memory ptr
			hw_memcpy(_mar_mem, rundata, MARGA_MEM_SIZE);
			mem_offset += MARGA_MEM_SIZE;
		}

		// prepare main FSM control loop

		// max bytes to copy into _mar_mem at a time (would block for this long)
		const unsigned max_bytes_to_copy = 128;

		// check execution state for issues periodically
		const unsigned execution_check_interval = 20;

		// delta thresholds for detecting buffer issues
		const int64_t warn_forward = (int64_t)(MARGA_MEM_SIZE / 4);
		const int64_t error_forward = (int64_t)(MARGA_MEM_SIZE / 2);

		// monitor buffer-low and buffer-underrun events
		unsigned mem_buffer_low = 0;
		bool mem_buffer_underrun = false;
		bool finished = false;

		// monitor RX-nearly-full and RX-full events
		unsigned rx_nearly_full = 0;
		bool rx_full = false;
		size_t rx_full_mem_loc = 0;

		// monitor output buffers
		uint32_t buf_full = 0, buf_err = 0;

		// monitor gradient issues
		bool ocra1_data_lost = false, ocra1_err = false, fhdo_err = false;

		// RX data — always accumulated into active_chunk
		unsigned rx_reads_per_loop = _min_rx_reads_per_loop;

		// RX streaming setup
		mpack_node_t stream_param = sa.request_param("stream_response");
		bool rx_streaming = mpack_node_type(stream_param) == mpack_type_bool
		        && mpack_node_bool(stream_param)
		        && sa.get_stream() != nullptr;

		const size_t chunk_threshold = 4096; // samples before swapping
		const size_t rx_chunk_pool_size = rx_streaming ? 4 : 1;
		std::vector<rx_chunk> rx_chunk_pool_storage(rx_chunk_pool_size);
		std::queue<rx_chunk *> rx_chunk_send_queue, rx_chunk_free_pool;
		std::mutex rx_stream_mtx;
		std::condition_variable rx_stream_cv;
		std::atomic<bool> rx_stream_done{false};
		std::atomic<mpack_error_t> rx_stream_error{mpack_ok};
		std::thread rx_streaming_thread;

		for (size_t i = 0; i < rx_chunk_pool_size; ++i) {
			rx_chunk_pool_storage[i].reserve(chunk_threshold * 2);
			rx_chunk_free_pool.push(&rx_chunk_pool_storage[i]);
		}
		rx_chunk *active_chunk = rx_chunk_free_pool.front();
		rx_chunk_free_pool.pop();

		// pool diagnostics
		size_t pool_low_watermark = rx_chunk_free_pool.size();
		unsigned pool_empty_count = 0;

		if (rx_streaming) {
			rx_streaming_thread = std::thread(rx_stream_thread,
			                                  sa.get_stream()->fd,
			                                  std::ref(rx_chunk_send_queue),
			                                  std::ref(rx_chunk_free_pool),
			                                  std::ref(rx_stream_mtx),
			                                  std::ref(rx_stream_cv),
			                                  std::ref(rx_stream_done),
			                                  std::ref(rx_stream_error));
		}

		// start the FSM
		wr32(_ctrl, 0x1);

		// main copying and reading loop
		unsigned execution_loops = 0;
		// track program counter execution
		int64_t total_pc = 0;
		size_t last_pc_hw = 0, old_pc = 0;
		while (not finished) {
			uint32_t exec = rd32(_exec);
			uint32_t state = exec >> 24;
			size_t pc_hw = (exec & 0xffffff) << 2; // convert to mem offset

			// Modular delta-accumulation: derive forward/backward
			// movement from the difference in raw PC values.
			// Values very close to MARGA_MEM_SIZE are treated
			// as backtracks, as sometimes the PC can go back a few instructions.
			// Everything else is forward.
			const uint32_t max_backtrack = 16;
			uint32_t raw_delta = (uint32_t)(pc_hw - last_pc_hw) & MARGA_MEM_MASK;
			int64_t delta;
			if (raw_delta >= MARGA_MEM_SIZE - max_backtrack) {
				delta = (int64_t)raw_delta - (int64_t)MARGA_MEM_SIZE; // backtrack
			} else {
				delta = (int64_t)raw_delta; // forward advance

				// Threshold checks on forward movement magnitude:
				// large deltas indicate the poll loop fell behind
				// the FPGA's execution rate.
				if (delta > error_forward) {
					mem_buffer_underrun = true;
					debug_printf("mem buf underrun: delta=%ld, pc_hw=%zu, last_pc_hw=%zu\n",
					             (long)delta, pc_hw, last_pc_hw);
					break;
				} else if (delta > warn_forward) {
					++mem_buffer_low;
					debug_printf("mem buf low: delta=%ld, pc_hw=%zu, last_pc_hw=%zu\n",
					             (long)delta, pc_hw, last_pc_hw);
				}
			}

			total_pc += delta;
			last_pc_hw = pc_hw;

			size_t pc = (size_t)total_pc;
			debug_printf("pc_hw %zu, last_pc_hw %zu, pc %zu, delta %ld\n", pc_hw, last_pc_hw, pc, (long)delta);

			size_t total_bytes_remaining = total_bytes_to_copy - mem_offset;
			int bytes_to_copy = 0;
			if (total_bytes_remaining != 0) {
				// check whether data needs copying in this round
				//
				// -max_backtrack to keep a buffer zone of un-copied
				// instructions before PC location
				bytes_to_copy = pc - mem_offset + MARGA_MEM_SIZE - max_backtrack;
				if (bytes_to_copy > (int)total_bytes_remaining) {
					bytes_to_copy = total_bytes_remaining;
				}
			}

			if (bytes_to_copy > 0) {
				if (bytes_to_copy > (int)max_bytes_to_copy) {
					// avoid starving the RX for time
					bytes_to_copy = max_bytes_to_copy;
				}

				size_t local_mem_offset = mem_offset & MARGA_MEM_MASK;

				// check whether this copy will wrap
				if (local_mem_offset + bytes_to_copy > MARGA_MEM_SIZE) { // wrapping: copy in two parts
					int first_bytes = MARGA_MEM_SIZE - local_mem_offset;
					int second_bytes = bytes_to_copy - first_bytes;
					hw_memcpy(_mar_mem + local_mem_offset,
					          rundata + mem_offset, first_bytes);
					mem_offset += first_bytes;
					hw_memcpy(_mar_mem, rundata + mem_offset, second_bytes);
					mem_offset += second_bytes;
				} else { // no wrapping
					debug_printf("hw_memcpy: %zu, %zu, %u\n", local_mem_offset, mem_offset, bytes_to_copy);
					[[maybe_unused]] auto k = (volatile char *)hw_memcpy(_mar_mem + local_mem_offset,
					                                                     rundata + mem_offset, bytes_to_copy);
					debug_printf("bytes copied: %zu\n", k - _mar_mem - local_mem_offset);
					mem_offset += bytes_to_copy;
				}
			}

			// Read out RX
			unsigned rx_locs = read_rx(active_chunk->rx0_i, active_chunk->rx0_q,
			                           active_chunk->rx1_i, active_chunk->rx1_q, rx_reads_per_loop);
			// Streaming: swap chunk to the send queue when big enough
			if (rx_streaming && active_chunk->largest_vector_size() >= chunk_threshold) {
				std::unique_lock<std::mutex> lock(rx_stream_mtx, std::try_to_lock);
				if (lock.owns_lock() && !rx_chunk_free_pool.empty()) {
					rx_chunk_send_queue.push(active_chunk);
					active_chunk = rx_chunk_free_pool.front();
					rx_chunk_free_pool.pop();
					size_t free_now = rx_chunk_free_pool.size();
					if (free_now < pool_low_watermark) {
						pool_low_watermark = free_now;
					}
					lock.unlock();
					rx_stream_cv.notify_one();
				} else if (lock.owns_lock()) {
					++pool_empty_count;
				}
				// else: lock contention — keep accumulating
			}
			// Simple dynamic FIFO read-out speed governor
			if (rx_locs > MARGA_RX_FIFO_SPACE - 2 * _max_rx_reads_per_loop) {
				rx_full = true;
				rx_full_mem_loc = old_pc;
				// desperately try to clear the FIFOs
				rx_reads_per_loop = _max_rx_reads_per_loop;
			} else if (rx_locs > 2 * MARGA_RX_FIFO_SPACE / 3) {
				++rx_nearly_full;
				// try hard to clear the FIFOs; scale up quickly
				if (rx_reads_per_loop < _max_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop * 4;
			} else if (rx_locs > MARGA_RX_FIFO_SPACE / 3) {
				if (rx_reads_per_loop < _max_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop * 2; // scale up
			} else {
				if (rx_reads_per_loop > _min_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop / 2;
			}

			// periodic buffer status checking
			if (execution_loops % execution_check_interval == 0) {
				buf_full = buf_full | rd32(_buf_full);
				buf_err = buf_err | rd32(_buf_err); // don't really need or here due to break; just for consistency
				if (buf_err) break;

				uint32_t status_latch = rd32(_status_latch);
				ocra1_data_lost = ocra1_data_lost | (status_latch & 0x1);
				ocra1_err = ocra1_err | (status_latch & 0x2);
				fhdo_err = fhdo_err | (status_latch & 0x4);
			}

			if (state == MAR_STATE_HALT) {
				finished = true;
			}
			if (rx_streaming) {
				mpack_error_t err = rx_stream_error.load();
				if (err != mpack_ok) {
					sa.add_error(std::string("RX stream write failed: ")
					             + mpack_error_to_string(err));
					break;
				}
			}
			old_pc = pc;
			++execution_loops;
		}

		// gracefully reset the FSM if no errors occurred
		if (finished) {
			wr32(_ctrl, 0x0);
			// emergency halt the FSM and reset the hardware
		} else {
			halt_and_reset(&sa);
		}

		// final buffer and gradient status checks
		buf_full = buf_full | rd32(_buf_full);
		buf_err = buf_err | rd32(_buf_err);

		uint32_t status_latch = rd32(_status_latch);
		ocra1_data_lost = ocra1_data_lost | (status_latch & 0x1);
		ocra1_err = ocra1_err | (status_latch & 0x2);
		fhdo_err = fhdo_err | (status_latch & 0x4);

		// post-mortem reporting
		if (mem_buffer_underrun) {
			sprintf(t, "memory buffer underrun: FPGA PC advanced >%zu bytes in one poll interval", (size_t)error_forward);
			sa.add_error(t);
		} else if (mem_buffer_low) {
			sprintf(t, "memory buffer low %u times: FPGA PC advanced >%zu bytes in one poll interval", mem_buffer_low, (size_t)warn_forward);
			sa.add_warning(t);
		} else if (mem_offset != total_bytes_to_copy) {
			sprintf(t, "didn't copy %zu bytes before FSM halted", total_bytes_to_copy - mem_offset);
			sa.add_error(t);
		}

		if (rx_full) {
			sprintf(t, "RX FIFO/s full during sequence around byte address 0x%0zx", rx_full_mem_loc);
			sa.add_error(t);
		} else if (rx_nearly_full) {
			sprintf(t, "RX FIFO/s almost filled during sequence %u times", rx_nearly_full);
			sa.add_warning(t);
		}

		if (buf_err) {
			sprintf(t, "output buffers overflowed during sequence: 0x%08x", buf_err);
			sa.add_error(t);
		} else if (buf_full) {
			sprintf(t, "output buffers were full during sequence: 0x%08x", buf_full);
			sa.add_warning(t);
		}

		if (ocra1_data_lost) sa.add_warning("ocra1 data was lost (overwritten before being sent)");
		if (ocra1_err) sa.add_error("ocra1 gradient error; possibly missing samples");
		if (fhdo_err) sa.add_error("gpa-fhdo gradient error; possibly missing samples");

		// Wait for any in-flight CIC pipeline samples to reach the
		// FIFOs, then read them out.  The CIC drain time is bounded
		// by decimation_rate * cic_stages / clock_freq; 2 ms is
		// conservative for all practical configurations.
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		while (read_rx(active_chunk->rx0_i, active_chunk->rx0_q,
		               active_chunk->rx1_i, active_chunk->rx1_q)) {}

		if (rx_streaming) {
			// Push final chunk if it has data
			if (active_chunk->largest_vector_size() > 0) {
				std::lock_guard<std::mutex> lock(rx_stream_mtx);
				rx_chunk_send_queue.push(active_chunk);
				active_chunk = nullptr;
				rx_stream_cv.notify_one();
			}
			// Signal thread to finish and wait
			rx_stream_done.store(true);
			rx_stream_cv.notify_one();
			rx_streaming_thread.join();

			sprintf(t, "rx chunk pool: size=%zu, low watermark=%zu, empty count=%u",
			        rx_chunk_pool_size, pool_low_watermark, pool_empty_count);
			sa.add_info(t);
		}

		// TODO make sure all the buffers and RX FIFOs are empty
		unsigned buf_empty = rd32(_buf_empty);
		if (buf_empty != MAR_BUF_ALL_EMPTY) {
			sprintf(t, "output buffers were not empty at the end of sequence: 0x%08x", buf_empty);
			sa.add_warning(t);
		}

		// wait a little longer for gradient interfaces to become idle
		unsigned gpa_idle_tries = 0;
		bool gpas_idle = false;
		while (gpa_idle_tries < _gpa_idle_tries_limit) {
			gpas_idle = (rd32(_status) & MAR_STATUS_GPA_MASK) == 0;
			if (gpas_idle) break;
			++gpa_idle_tries;
		}
		if (not gpas_idle) {
			sprintf(t, "GPAs not idle at the end of sequence");
			sa.add_warning(t);
		}

		// halt();

		// encode the RX replies
		if (rx_streaming) {
			// Data was already streamed as chunks; reply just confirms success
			mpack_write(wr, c_ok);
		} else {
			unsigned rx0_elem = active_chunk->rx0_i.size(), rx1_elem = active_chunk->rx1_i.size();
			if (!rx0_elem and !rx1_elem) {
				mpack_write(wr, c_ok);
				sprintf(t, "no RX data received");
				sa.add_warning(t);
			} else {
				mpack_start_map(wr, (rx0_elem ? 2 : 0) + (rx1_elem ? 2 : 0));
				if (rx0_elem) {
					mpack_write_cstr(wr, "rx0_i");
					mpack_start_array(wr, rx0_elem);
					for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, active_chunk->rx0_i[k]);
					mpack_finish_array(wr);
					mpack_write_cstr(wr, "rx0_q");
					mpack_start_array(wr, rx0_elem);
					for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, active_chunk->rx0_q[k]);
					mpack_finish_array(wr);
				}

				if (rx1_elem) {
					mpack_write_cstr(wr, "rx1_i");
					mpack_start_array(wr, rx1_elem);
					for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, active_chunk->rx1_i[k]);
					mpack_finish_array(wr);
					mpack_write_cstr(wr, "rx1_q");
					mpack_start_array(wr, rx1_elem);
					for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, active_chunk->rx1_q[k]);
					mpack_finish_array(wr);
				}

				mpack_finish_map(wr);
			}
		}
	}

	// Test client-server network throughput
	auto tln = sa.get_command_and_start_reply("test_net", status);
	if (status == 1) {
		++commands_understood;
		unsigned data_size = mpack_node_uint(tln);

		mpack_start_map(wr, 2); // Two elements in map
		mpack_write_cstr(wr, "array1");
		mpack_start_array(wr, data_size);
		for (unsigned k{0}; k < data_size; ++k) mpack_write(wr, 1.01 * k); // generic, needs C11
		mpack_finish_array(wr);

		mpack_write_cstr(wr, "array2");
		mpack_start_array(wr, data_size);
		for (unsigned k{0}; k < data_size; ++k) mpack_write(wr, 1.01 * (k + 10)); // generic, needs C11
		mpack_finish_array(wr);

		mpack_finish_map(wr);
	} else if (status == -1) {
		// TODO: callback or similar
	}

	// Test bus throughput
	auto tbt = sa.get_command_and_start_reply("test_bus", status);
	if (status == 1) {
		++commands_understood;
		auto n_tests = mpack_node_u32(tbt);

		auto start_t = std::chrono::system_clock::now();
		unsigned m = 0;
		for (unsigned k = 0; k < n_tests; ++k) {
			m += k;
		}
		auto null_t = std::chrono::system_clock::now();

		for (unsigned k = 0; k < n_tests; ++k) {
			m += rd32(_status); // repeatedly read the register
		}
		auto read_t = std::chrono::system_clock::now();

		for (unsigned k = 0; k < n_tests; ++k) {
			wr32(_ctrl, k & 0xfffffffc); // repeatedly write the register, but avoid setting the lower 2 bits
		}

		wr32(_ctrl, m & 0xfffffffc); // avoid m getting optimised out of the first loop

		auto write_t = std::chrono::system_clock::now();

		// reply will contain the three differences
		int64_t null_ti = std::chrono::duration_cast<std::chrono::microseconds>(null_t - start_t).count(),
		        read_ti = std::chrono::duration_cast<std::chrono::microseconds>(read_t - null_t).count(),
		        write_ti = std::chrono::duration_cast<std::chrono::microseconds>(write_t - read_t).count();

		mpack_start_array(wr, 3);
		mpack_write(wr, null_ti);
		mpack_write(wr, read_ti);
		mpack_write(wr, write_ti);
		mpack_finish_array(wr);
	}

	// Determine if the server is running on hardware or just an emulation
	sa.get_command_and_start_reply("are_you_real", status);
	if (status == 1) {
		++commands_understood;
#ifdef __arm__
		mpack_write(wr, "hardware");
#else

#ifdef VERILATOR_BUILD
		mpack_write(wr, "simulation");
#else
		mpack_write(wr, "software");
#endif

#endif
	}

	// Final housekeeping
	mpack_finish_map(wr);

	if (commands_understood != commands_present) {
		assert(commands_understood <= commands_present && "Serious bug in logic");
		sa.add_error("not all client commands were understood");

		// Fill in remaining elements of the response map (TODO: maybe make this more sophisticated?)
		while (commands_present != 0) {
			char t[100];
			sprintf(t, "UNKNOWN%zu", commands_present);
			mpack_write_kv(wr, t, -1);
			commands_present--;
		}
	}

	return problems;
}

void hardware::init_mem() {
#ifdef __arm__
	int fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		throw hw_error("failed to access memory device - check sudo permissions and/or platform");
	}
#else
	char tempfile[100] = "/tmp/marcos_server_mem";

	int fd = open(tempfile, O_RDWR);
	if (fd < 0) {
		char errstr[1024];
		size_t filesize_KiB = 4 * END_OFFSET / EMU_PAGESIZE; // 4 because 4 KiB / page
		sprintf(errstr, "Failed to open simulated memory device.\n"
		                "Check whether %s exists, and if not create it using:\n"
		                "fallocate -l %ldKiB %s",
		        tempfile, filesize_KiB, tempfile);
		throw hw_error(errstr);
	}
#endif

	// set up shared memory (please refer to the memory offset table)

	// VN: I'm not sure why in the original server, different data
	// types were used for some of these. Perhaps to allow
	// different access widths?
	_slcr = (uint32_t *)mmap(NULL, SLCR_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, SLCR_OFFSET);
	_mar_base = (uint32_t *)mmap(NULL, MARGA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MARGA_OFFSET);

	// Map the control and status registers
	_ctrl = _mar_base + 0;
	// slv_reg1 for extension in the future
	_direct = _mar_base + 2;
	// slv_reg3 for extension in the future
	_exec = _mar_base + 4; // execution information
	_status = _mar_base + 5; // external status, ADC etc
	_status_latch = _mar_base + 6; // latched external status
	_buf_err = _mar_base + 7; // latched buffer errors
	_buf_full = _mar_base + 8; // latched full buffers
	_buf_empty = _mar_base + 9; // empty buffers
	_rx_locs = _mar_base + 10; // RX data available
	_rx0_i_data = _mar_base + 11; // RX0 i data
	_rx1_i_data = _mar_base + 12; // RX1 i data
	_rx0_q_data = _mar_base + 13; // RX0 q data
	_rx1_q_data = _mar_base + 14; // RX1 q data

	// /2 since mem is halfway in address space, /4 to convert to 32-bit instead of byte addressing
	_mar_mem = reinterpret_cast<volatile char *>(_mar_base + MARGA_SIZE / 2 / 4);
}

void hardware::halt() {
	// Assert halt (stop_fsm = bit 1).  Keep run_fsm (bit 0) clear so
	// that HALT state will transition to IDLE once we get there.
	wr32(_ctrl, 0x2);

	// Wait for the FSM to reach HALT.  States like COUNTDOWN, TRIG
	// and RUN don't check stop_fsm, so we must wait for them to
	// finish naturally until the FSM re-enters PREPARE, which is
	// the only state that checks stop_fsm and transitions to HALT.
	unsigned k = 0;
	while (k < _halt_tries_limit) {
		uint32_t state = rd32(_exec) >> 24;
		if (state == MAR_STATE_HALT || state == MAR_STATE_IDLE) break;
		++k;
	}

	// Now release halt so the FSM transitions HALT → IDLE.
	// The HDL only processes direct buffer writes in IDLE.
	wr32(_ctrl, 0x0);

	// Wait for IDLE
	while (k < _halt_tries_limit) {
		if ((rd32(_exec) >> 24) == MAR_STATE_IDLE) break;
		++k;
	}
	if ((rd32(_exec) >> 24) != MAR_STATE_IDLE) {
		fprintf(stderr, "halt: FSM did not reach IDLE after %u tries (state=0x%02x)\n",
		        k, rd32(_exec) >> 24);
	}

	// Now in IDLE: turn off readout via direct write to buffer 16
	unsigned buf = 16; // buffer 16 = RX ctrl buffer index
	unsigned val = 0x0000; // halt the RX
	wr32(_direct, (buf << 24) | (val & 0xffff));

	// Wait a while for all the output buffers to empty
	while (k < _halt_tries_limit) {
		if (rd32(_buf_empty) == MAR_BUF_ALL_EMPTY) break;
		++k;
	}

	// Empty RX FIFOs (do this last)
	discard_rx();
}

void hardware::halt_and_reset(server_action *sa) {
	// Old OCRA server comment: set FPGA clock to 143 MHz (VN: not
	// sure how this works - probably not needed any more?)
	_slcr[2] = 0xDF0D;
	_slcr[92] = (_slcr[92] & ~0x03F03F30) | 0x00100700;

	halt();

	// Zero the TX I/Q outputs directly. Buffers 5..8 are TX0_I, TX0_Q,
	// TX1_I, TX1_Q (see the buffer index constants in
	// marcos_client/marmachine.py; marcompile.col2buf maps DSP columns
	// onto those same indices).
	for (unsigned tx_buf = 5; tx_buf <= 8; ++tx_buf) {
		wr32(_direct, (tx_buf << 24) | 0x0000);
	}

	// Park the gradient DACs at their per-board zero-current code so
	// the amplifier isn't left driving a static current after a cancel.
	// The words are board-specific (OCRA1 uses 0x...10000 frames at DAC
	// code 0; GPA-FHDO uses 0x...8000 frames at DAC code 0x8000) and
	// must be registered ahead of time by the client via the
	// set_gpa_zero_words RPC.
	if (!_gpa_zero_words.empty()) {
		unsigned timeouts = 0;
		for (uint32_t word : _gpa_zero_words) {
			if (!write_gpa_word_direct(word)) ++timeouts;
		}
		if (timeouts) {
			char msg[240];
			snprintf(msg, sizeof(msg),
			         "halt_and_reset: GPA serialiser stayed busy after %u of %zu"
			         " zero-word writes (%u polls per write); the gradient DAC"
			         " may not have been parked at zero.",
			         timeouts, _gpa_zero_words.size(), _gpa_idle_tries_limit);
			fprintf(stderr, "%s\n", msg);
			if (sa) sa->add_warning(msg);
		}
	} else {
		// Old client that never registered the zero words: we cannot
		// safely zero the DACs from here, so the last sample clocked
		// out of marga stays latched on the gradient amplifier. On a
		// cancel mid-sequence that's a sustained DC into the coil and
		// will thermally trip the amp.
		char ts[32];
		time_t now = time(nullptr);
		struct tm tm_now;
		localtime_r(&now, &tm_now);
		strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_now);
		fprintf(stderr,
		        "[%s] halt_and_reset: _gpa_zero_words is empty -- client "
		        "did not call set_gpa_zero_words; gradient DACs will "
		        "remain latched at their last sample. Update the "
		        "client (marcos_client Experiment.__init__) to send "
		        "the zero words after init_hw().\n", ts);
	}
}

bool hardware::write_gpa_word_direct(uint32_t word) {
	// MSB to buffer 2 first, then LSB to buffer 1: the LSB write is
	// what strobes the gradient SPI serialiser (see OCRA1.init_hw in
	// marcos_client/grad_board.py). Reversing the order would either
	// double-clock the serialiser or send a half-formed frame.
	unsigned grad_msb_buf = 2, grad_lsb_buf = 1;
	wr32(_direct, (grad_msb_buf << 24) | ((word >> 16) & 0xffff));
	wr32(_direct, (grad_lsb_buf << 24) | (word & 0xffff));

	for (unsigned k = 0; k < _gpa_idle_tries_limit; ++k) {
		if ((rd32(_status) & MAR_STATUS_GPA_MASK) == 0) return true;
	}
	return false;
}

unsigned hardware::read_rx(std::vector<uint32_t> &rx0_i, std::vector<uint32_t> &rx0_q,
                           std::vector<uint32_t> &rx1_i, std::vector<uint32_t> &rx1_q,
                           const unsigned max_reads) {
	uint32_t rxlocs = rd32(_rx_locs);
	int fifo0_locs = rxlocs & 0xffff, fifo1_locs = rxlocs >> 16;

	unsigned reads = 0;

	debug_printf("initial fifo locs: %d, %d\n", fifo0_locs, fifo1_locs);

	while (fifo0_locs > 0 or fifo1_locs > 0) {
		if (reads >= max_reads) break;

		bool read_fifo0 = false, read_fifo1 = false;
		if (fifo1_locs > 2 * fifo0_locs) {
			// too much data in fifo1, read it exclusively
			read_fifo1 = true;
		} else if (fifo0_locs > 2 * fifo1_locs) {
			// too much data in fifo0, read it exclusively
			read_fifo0 = true;
		} else { // read both
			read_fifo0 = true;
			read_fifo1 = true;
		}

		if (read_fifo0) { // read pair of samples
			rx0_q.push_back(rd32(_rx0_q_data)); // read q first
			rx0_i.push_back(rd32(_rx0_i_data)); // pop fifo
			++reads;
			--fifo0_locs;
		}
		if (read_fifo1) { // read pair of samples
			rx1_q.push_back(rd32(_rx1_q_data)); // read q first
			rx1_i.push_back(rd32(_rx1_i_data)); // pop fifo
			++reads;
			--fifo1_locs;
		}
	}

	// Let some time pass so that the FIFO-fullness register can update
	// (this is purely to make the simulation behaviour more realistic)
#ifdef VERILATOR_BUILD
	rd32(_rx1_q_data);
#endif

	// return the FIFO closest to filling
	if (fifo0_locs > fifo1_locs) return fifo0_locs;
	else return fifo1_locs;
}

void hardware::discard_rx() {
	// RX chain should already be stopped (halt() disables RX enable
	// and resets the CIC via buffer 16).  Drain any remaining
	// samples from the FIFOs.
	uint32_t rxlocs;
	while ((rxlocs = rd32(_rx_locs)) != 0) {
		int fifo0_locs = rxlocs & 0xffff, fifo1_locs = rxlocs >> 16;
		while (fifo0_locs > 0 || fifo1_locs > 0) {
			if (fifo0_locs > 0) {
				rd32(_rx0_q_data);
				rd32(_rx0_i_data);
				--fifo0_locs;
			}
			if (fifo1_locs > 0) {
				rd32(_rx1_q_data);
				rd32(_rx1_i_data);
				--fifo1_locs;
			}
		}
	}
}

void hardware::wr32(volatile uint32_t *addr, uint32_t data) {
#ifdef VERILATOR_BUILD
	if (addr >= _mar_base && addr < _mar_base + MARGA_SIZE) {
		// do byte-address arithmetic
		auto offs_addr = reinterpret_cast<volatile char *>(addr)
		        - reinterpret_cast<volatile char *>(_mar_base);
		// printf("addresses 0x%0lx, 0x%0lx\n", addr, _mar_base);
		// printf("write mar 0x%0lx, 0x%08x\n", offs_addr, data);
		mm->wr32(offs_addr, data); // convert to byte addressing
	} else {
		printf("write addr 0x%0lx, 0x%08x NOT SIMULATED\n", (size_t)addr, data);
	}
#else
	*addr = data;
#endif
}

uint32_t hardware::rd32(volatile uint32_t *addr) {
#ifdef VERILATOR_BUILD
	if (addr >= _mar_base && addr < _mar_base + MARGA_SIZE) {
		// do byte-address arithmetic
		auto offs_addr = reinterpret_cast<volatile char *>(addr)
		        - reinterpret_cast<volatile char *>(_mar_base);
		// printf("read mar 0x%0lx\n", offs_addr);
		return mm->rd32(offs_addr); // convert to byte addressing
	} else {
		printf("read addr 0x%0lx NOT SIMULATED\n", (size_t)addr);
		return 0;
	}
#else
	return *addr;
#endif
}

volatile void *hardware::hw_memcpy(volatile void *s1, const void *s2, size_t n) {
	assert(n % 4 == 0 && "hw_memcpy: size must be a multiple of 4 (FPGA word size)");

	// Use individual 32-bit volatile writes rather than memcpy.
	// At high optimisation levels, memcpy can be lowered to wide
	// (e.g. NEON) stores whose AXI bus transactions silently
	// corrupt data written to the FPGA BRAM.
	auto *d = reinterpret_cast<volatile uint32_t *>(s1);
	auto *s = reinterpret_cast<const uint32_t *>(s2);
	size_t nu = n / 4;
	while (nu-- != 0) {
		wr32(d++, *s++);
	}
	return d;
}

size_t hardware::hw_mpack_node_copy_data(mpack_node_t node, volatile char *buffer, size_t bufsize) {
	// Copy via a temporary buffer + individual 32-bit volatile writes,
	// to avoid wide stores that corrupt FPGA BRAM (same issue as hw_memcpy).
	std::vector<char> tmp(bufsize);
	size_t bytes_copied = mpack_node_copy_data(node, tmp.data(), bufsize);
	hw_memcpy(buffer, tmp.data(), bytes_copied);
	return bytes_copied;
}

/// Each chunk is: [marcos_rx_chunk(3), chunk_index, rx0_i[], rx0_q[], rx1_i[], rx1_q[]]
void hardware::rx_stream_thread(int fd,
                                std::queue<rx_chunk *> &send_queue,
                                std::queue<rx_chunk *> &free_pool,
                                std::mutex &mtx,
                                std::condition_variable &cv,
                                std::atomic<bool> &done,
                                std::atomic<mpack_error_t> &error) {
	const size_t buf_size = 1024 * 1024; // 1MB serialization buffer
	std::vector<char> buf(buf_size);
	uint32_t chunk_index = 0;

	mpack_writer_t w;
	mpack_writer_init(&w, buf.data(), buf_size);
	stream_t stream = {fd};
	mpack_writer_set_context(&w, &stream);
	mpack_writer_set_flush(&w, &write_stream);

	auto write_vec = [&](const std::vector<uint32_t> &v) {
		mpack_start_array(&w, v.size());
		for (auto val: v) mpack_write_int(&w, val);
		mpack_finish_array(&w);
	};

	while (true) {
		rx_chunk *chunk = nullptr;
		{
			std::unique_lock<std::mutex> lock(mtx);
			cv.wait(lock, [&] { return !send_queue.empty() || done.load(); });
			if (send_queue.empty() && done.load()) {
				break;
			}
			chunk = send_queue.front();
			send_queue.pop();
		}

		mpack_start_array(&w, 3);
		mpack_write_u32(&w, marcos_rx_chunk);
		mpack_write_u32(&w, chunk_index++);

		unsigned rx0_elem = chunk->rx0_i.size(), rx1_elem = chunk->rx1_i.size();
		mpack_start_map(&w, (rx0_elem ? 2 : 0) + (rx1_elem ? 2 : 0));
		if (rx0_elem) {
			mpack_write_cstr(&w, "rx0_i");
			write_vec(chunk->rx0_i);
			mpack_write_cstr(&w, "rx0_q");
			write_vec(chunk->rx0_q);
		}
		if (rx1_elem) {
			mpack_write_cstr(&w, "rx1_i");
			write_vec(chunk->rx1_i);
			mpack_write_cstr(&w, "rx1_q");
			write_vec(chunk->rx1_q);
		}
		mpack_finish_map(&w);

		mpack_finish_array(&w);
		mpack_writer_flush_message(&w);

		if (mpack_writer_error(&w) != mpack_ok) {
			error.store(mpack_writer_error(&w));
			break;
		}

		// Return chunk to free pool
		chunk->clear();
		{
			std::lock_guard<std::mutex> lock(mtx);
			free_pool.push(chunk);
		}
	}

	mpack_writer_destroy(&w);
}
