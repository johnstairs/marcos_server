/** @file hardware.hpp
    @brief Hardware management and interface/data transfer to the Zynq PL.
*/

#ifndef _HARDWARE_HPP_
#define _HARDWARE_HPP_

#include <inttypes.h> // TODO is this the right include?
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include "mpack/mpack.h"

// Memory-mapped device sizes
static const unsigned PAGESIZE = sysconf(_SC_PAGESIZE); // should be 4096 (4KiB) on both x86_64 and ARM
static const unsigned SLCR_SIZE = PAGESIZE,
                      MARGA_SIZE = 128 * PAGESIZE,
                      MARGA_MEM_SIZE = 64 * PAGESIZE;
static const unsigned MARGA_MEM_MASK = 0x3ffff;
static const unsigned MARGA_RX_FIFO_SPACE = 16384;

// marga internal states
static const unsigned MAR_STATE_IDLE = 0, MAR_STATE_PREPARE = 1, MAR_STATE_RUN = 2,
                      MAR_STATE_COUNTDOWN = 3, MAR_STATE_TRIG = 4, MAR_STATE_TRIG_FOREVER = 5,
                      MAR_STATE_HALT = 8;

static const unsigned MAR_BUFS = 24;
static const unsigned MAR_BUF_ALL_EMPTY = (1 << MAR_BUFS) - 1;

static const unsigned MAR_STATUS_GPA_MASK = 0x00030000;

class server_action;
struct stream_t;

/// @brief Pre-allocated RX data buffer for streaming
struct rx_chunk {
	std::vector<uint32_t> rx0_i, rx0_q, rx1_i, rx1_q;
	void clear() {
		rx0_i.clear();
		rx0_q.clear();
		rx1_i.clear();
		rx1_q.clear();
	}

	size_t largest_vector_size() const {
		return std::max({rx0_i.size(), rx0_q.size(), rx1_i.size(), rx1_q.size()});
	}

	void reserve(size_t n) {
		rx0_i.reserve(n);
		rx0_q.reserve(n);
		rx1_i.reserve(n);
		rx1_q.reserve(n);
	}
};

class hardware {
public:
	hardware();
	~hardware();

	int run_request(server_action &sa);

	/// @brief Set up shared memory, control registers etc; these
	/// aspects are not client-configurable. If compiled on x86,
	/// just mimics the shared memory.
	void init_mem();

	/// @brief Halt the FSM, interrupting any ongoing sequence and/or readout in progress
	void halt();

	/// @brief Halt and reset all outputs to default values, even
	/// if the cores are currently running. Activated when an
	/// emergency stop command arrives. If @p sa is non-null, any
	/// diagnostics raised while zeroing the gradient DACs (e.g. a
	/// GPA serialiser that fails to go idle within
	/// ``_gpa_idle_tries_limit`` polls) are surfaced to the client
	/// via ``server_action::add_warning`` in addition to being
	/// logged on stderr.
	void halt_and_reset(server_action *sa = nullptr);
private:
	/// @brief Pre-computed 32-bit direct-write words that drive
	/// every gradient DAC channel to its zero-current code. The
	/// client must register these via the set_gpa_zero_words RPC
	/// after configuring the gradient board, because the encoding
	/// (DAC midpoint, channel/broadcast bits, board-specific
	/// framing) is only known to the client-side grad_board
	/// implementation. Empty until the client populates it; in
	/// that case halt_and_reset() cannot safely zero the DACs and
	/// will log a warning.
	///
	/// Lifetime is **process-lifetime, not per-connection**: the
	/// vector lives on the singleton ``hardware`` instance and
	/// persists across client disconnects/reconnects for as long
	/// as the marcos_server process is alive. This is why most
	/// Monarch ``Experiment`` invocations can rely on an earlier
	/// ``InitGpas`` (or any prior ``Experiment(init_gpa=True)``)
	/// having registered the words; subsequent experiments that
	/// pass ``init_gpa=False`` inherit the same vector. The words
	/// are only lost when the server process exits, the FPGA is
	/// re-flashed, or a new ``set_gpa_zero_words`` RPC clears and
	/// rewrites the vector (e.g. when switching gradient boards).
	std::vector<uint32_t> _gpa_zero_words;

	/// @brief Issue a single 32-bit gradient-serialiser word as a
	/// direct write, bypassing marga timing. Writes the MSB half
	/// to buffer 2 (GRAD_MSB) first, then the LSB half to buffer
	/// 1 (GRAD_LSB); the LSB write is what strobes the SPI
	/// serialiser (see marcos_client/grad_board.py OCRA1.init_hw)
	/// so the ordering is load-bearing. After the LSB write,
	/// polls MAR_STATUS_GPA_MASK up to _gpa_idle_tries_limit
	/// times waiting for the serialiser to drop busy.
	///
	/// @return true if the serialiser went idle within the poll
	/// limit, false on timeout (the caller is responsible for
	/// reporting the failure).
	bool write_gpa_word_direct(uint32_t word);

	// Config variables
	unsigned _read_tries_limit = 1000; // retry attempts for each data sample
	unsigned _halt_tries_limit = 1000000; // read retry attemps for HALT state at the end of the sequence
	unsigned _gpa_idle_tries_limit = 1000; // how long to wait for GPA interfaces to become idle at the end of a sequence
	unsigned _min_rx_reads_per_loop = 16;
	unsigned _max_rx_reads_per_loop = 1024;

	// Peripheral register addresses in PL
	volatile uint32_t *_slcr, *_mar_base, *_ctrl, *_direct, *_exec, *_status,
	        *_status_latch, *_buf_err, *_buf_full, *_buf_empty, *_rx_locs,
	        *_rx0_i_data, *_rx1_i_data, *_rx0_q_data, *_rx1_q_data;

	volatile char *_mar_mem;

	/// @brief Write

	/// @brief Read out RX FIFOs into vectors. Reads out either
	/// all the data available (default), or just the number of
	/// reads specified by max_reads. Doesn't strictly respect
	/// max_reads, only within ~4 reads. Returns the amount of 32b
	/// data in the most-filled FIFO, *before* reading began.
	unsigned read_rx(std::vector<uint32_t> &rx0_i, std::vector<uint32_t> &rx0_q,
	                 std::vector<uint32_t> &rx1_i, std::vector<uint32_t> &rx1_q,
	                 const unsigned max_reads = 100000);

	/// @brief Discard all samples currently in the RX FIFOs.
	/// The RX chain must already be stopped (e.g. via halt()).
	void discard_rx();

	/// @brief Background thread: serializes rx_chunks as msgpack
	/// messages and writes them to the client socket.
	static void rx_stream_thread(int fd,
	                             std::queue<rx_chunk *> &send_queue,
	                             std::queue<rx_chunk *> &free_pool,
	                             std::mutex &mtx,
	                             std::condition_variable &cv,
	                             std::atomic<bool> &done,
	                             std::atomic<mpack_error_t> &error);

	// methods to support simulation; most efficient to inline them
	inline void wr32(volatile uint32_t *addr, uint32_t data);
	inline uint32_t rd32(volatile uint32_t *addr);
	volatile void *hw_memcpy(volatile void *s1, const void *s2, size_t n);
	size_t hw_mpack_node_copy_data(mpack_node_t node, volatile char *buffer, size_t bufsize);
};

#endif
