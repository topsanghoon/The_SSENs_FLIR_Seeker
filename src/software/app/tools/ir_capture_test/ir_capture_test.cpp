#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <atomic>
#include <csignal>

#include <opencv2/opencv.hpp>

// === Test target threads and related types ===
#include "threads_includes/IR_CaptureThread.hpp"
#include "threads_includes/IR_TxThread.hpp"
#include "components/includes/IR_Frame.hpp"
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_quit{false};

void signal_handler(int signum) {
    std::cout << "\nCaught signal " << signum << ", shutting down..." << std::endl;
    g_quit.store(true);
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "=== IR Capture & Stream Test ===\n";
    std::cout << "Capturing from /dev/spidev1.0 (FLIR Lepton 2.5) and streaming to udp://192.168.0.15:5000\n";
    std::cout << "Press Ctrl+C to stop...\n\n";

    try {
        // Frame mailbox connecting capture and tx threads
        flir::SpscMailbox<std::shared_ptr<flir::IRFrameHandle>> frame_mailbox(2);

        // Configure IR capture for /dev/spidev1.0
        flir::IRCaptureConfig capture_config;
        
        // Configure IR TX for streaming to 192.168.0.15:5000
        flir::IR_TxThread::GstConfig gst_config;
        gst_config.pc_ip = "192.168.0.15";
        gst_config.port = 5000;
        gst_config.width = 80;   // Lepton 2.5
        gst_config.height = 60;  // Lepton 2.5
        gst_config.fps = 9;

        // Create TX thread first to get wake handle for capture thread
        flir::IR_TxThread tx_thread("IR_Tx", frame_mailbox, gst_config);
        auto wake_handle = tx_thread.create_wake_handle();
        
        // Create capture thread with wake handle
        flir::IR_CaptureThread capture_thread("IR_Capture", frame_mailbox, std::move(wake_handle), capture_config);

        // Start both threads
        capture_thread.start();
        tx_thread.start();

        std::cout << "[INFO] Pipeline started. Capturing from FLIR Lepton 2.5 and streaming...\n";
        std::cout << "[INFO] View stream with: gst-launch-1.0 udpsrc port=5000 ! "
                  << "application/x-rtp,encoding-name=JPEG,payload=26 ! "
                  << "rtpjpegdepay ! jpegdec ! videoconvert ! autovideosink\n\n";

        // Simple monitoring loop
        auto start_time = std::chrono::steady_clock::now();
        while (!g_quit.load()) {
            std::this_thread::sleep_for(2s);
            
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time).count();
            
            std::cout << "[STATS @ " << elapsed << "s] "
                      << "Frames: " << capture_thread.get_frame_count()
                      << ", Errors: " << capture_thread.get_error_count()
                      << ", Discards: " << capture_thread.get_discard_count()
                      << std::endl;
        }

        // Clean shutdown
        std::cout << "\n[INFO] Stopping...\n";
        capture_thread.stop();
        tx_thread.stop();
        capture_thread.join();
        tx_thread.join();
        
        std::cout << "[INFO] Final statistics:\n";
        std::cout << "  Total frames captured: " << capture_thread.get_frame_count() << std::endl;
        std::cout << "  Total errors: " << capture_thread.get_error_count() << std::endl;
        std::cout << "  Total discards: " << capture_thread.get_discard_count() << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
