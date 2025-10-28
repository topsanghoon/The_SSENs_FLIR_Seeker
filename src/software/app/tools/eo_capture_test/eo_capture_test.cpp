#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <atomic>
#include <csignal>

#include <opencv2/opencv.hpp>

// === Test target threads and related types ===
#include "threads_includes/EO_CaptureThread.hpp"
#include "threads_includes/EO_TxThread.hpp"
#include "components/includes/EO_Frame.hpp"
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
    
    std::cout << "=== EO Capture & Stream Test ===\n";
    std::cout << "Capturing from /dev/video0 and streaming to udp://192.168.0.15:5001\n";
    std::cout << "Press Ctrl+C to stop...\n\n";

    try {
        // Frame mailbox connecting capture and tx threads
        flir::SpscMailbox<std::shared_ptr<flir::EOFrameHandle>> frame_mailbox(2);

        // Configure capture for /dev/video0 -> 192.168.0.15:5001
        flir::EOCaptureConfig capture_config;
        
        flir::EO_TxThread::GstConfig gst_config;

        // Create TX thread first to get wake handle
        flir::EO_TxThread tx_thread("EO_Tx", frame_mailbox, gst_config);
        auto wake_handle = tx_thread.create_wake_handle();
        
        // Create capture thread with wake handle
        flir::EO_CaptureThread capture_thread("EO_Capture", frame_mailbox, std::move(wake_handle), capture_config);

        tx_thread.start();
        capture_thread.start();

        std::cout << "[INFO] Pipeline started. Capturing and streaming...\n";

        // Simple monitoring loop
        while (!g_quit.load()) {
            std::this_thread::sleep_for(2s);
            std::cout << "[STATS] Frames: " << capture_thread.get_frame_count() 
                      << ", Errors: " << capture_thread.get_error_count() << std::endl;
        }

        // Clean shutdown
        std::cout << "\n[INFO] Stopping...\n";
        capture_thread.stop();
        tx_thread.stop();
        capture_thread.join();
        tx_thread.join();
        
        std::cout << "[INFO] Total frames: " << capture_thread.get_frame_count() << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}