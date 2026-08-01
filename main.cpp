#include "sensor.hpp"
#include <memory>
#include <iostream>

int main() {
    std::cout << "--- Booting CPS Engine ---\n";
    
    // We create a scope block to prove the destructor works automatically
    {
        // 📦 Create Sensor exclusively owned by a smart pointer
        std::unique_ptr<Sensor> lidar = std::make_unique<Sensor>("LiDAR_V1");
        
        // Read the data
        lidar->readData();
        
    } // <-- lidar goes out of scope here. The Destructor triggers automatically!

    std::cout << "--- Engine Shutting Down ---\n";
    return 0;
}