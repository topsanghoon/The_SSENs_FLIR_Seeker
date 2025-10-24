#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>
#include <signal.h>
#include <iomanip>

// Net_RxThread and related includes
#include "threads_includes/Net_RxThread.hpp"
#include "ipc/event_bus_impl.hpp"
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"

// Global flag for clean shutdown
static std::atomic<bool> g_quit{false};

// Signal handler for clean shutdown (Ctrl+C)
void signal_handler(int signum) {
    std::cout << "\nCaught signal " << signum << ", shutting down..." << std::endl;
    g_quit.store(true);
}

int main(int argc, char** argv) {
    std::cout << "=== Net_RxThread Test ===" << std::endl;
    std::cout << "This test only focuses on receiving click commands from Windows PC" << std::endl;

    // Set up signal handler for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        // ============================
        //  Setup Pipeline Components
        // ============================

        // [Click Command Mailbox] - Net_RxThread produces, this test consumes
        flir::SpscMailbox<flir::UserCmd> mb_click(32);

        // [Event Bus] - For any events (though Net_RxThread doesn't publish much)
        flir::EventBus bus;

        // [Net_RxThread Configuration]
        flir::NetRxConfig config;
        config.port = 5001;  // Same port as Windows client sends to
        config.buffer_size = 1024;
        config.timeout_ms = 100;

        std::cout << "[CONFIG] Listening on UDP port: " << config.port << std::endl;

        // [Net_RxThread] - The star of this test
        flir::Net_RxThread net_thread(mb_click, config);

        // ============================
        //  Start Net_RxThread
        // ============================
        
        std::cout << "[START] Starting Net_RxThread..." << std::endl;
        net_thread.start();
        std::cout << "[INFO] Net_RxThread is running and listening for clicks" << std::endl;
        std::cout << "[INFO] Send clicks from Windows client to see them here" << std::endl;
        std::cout << std::endl;

        // ============================
        //  Main Loop - Process Click Commands
        // ============================
        
        uint32_t total_clicks = 0;
        auto start_time = std::chrono::steady_clock::now();

        while (!g_quit.load()) {
            // Check for incoming click commands
            if (auto cmd = mb_click.exchange(nullptr)) {
                total_clicks++;
                
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - start_time).count();

                std::cout << "[CLICK #" << total_clicks << "] "
                          << "seq=" << cmd->seq << " "
                          << "type=" << static_cast<int>(cmd->type) << " "
                          << "box=(" << std::fixed << std::setprecision(1) 
                          << cmd->box.x << ", " << cmd->box.y << ", " 
                          << cmd->box.width << ", " << cmd->box.height << ") "
                          << "elapsed=" << elapsed << "ms"
                          << std::endl;

                // Optional: Add some basic validation
                if (cmd->box.x < 0 || cmd->box.y < 0) {
                    std::cout << "  [WARNING] Invalid coordinates received!" << std::endl;
                }
                
                if (cmd->seq == 0) {
                    std::cout << "  [INFO] This might be an initialization click" << std::endl;
                }
            }

            // Small delay to prevent busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // ============================
        //  Cleanup
        // ============================
        
        std::cout << std::endl;
        std::cout << "[STOP] Stopping Net_RxThread..." << std::endl;
        net_thread.stop();
        net_thread.join();

        std::cout << "[STATS] Total clicks received: " << total_clicks << std::endl;
        std::cout << "[DONE] Net_RxThread test completed successfully" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}