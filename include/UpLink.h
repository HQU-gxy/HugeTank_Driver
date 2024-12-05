#pragma once

#include <utility>

namespace UpLink
{
    /**
     * @brief Initialize the UpLink UART and start the command reading timer
     */
    void init();

    /**
     * @brief Check if there is a new command available
     *
     * @return true if there is a new command available
     */
    bool cmdAvailable();

    /**
     * @brief Get the target speed from the latest command
     *
     * @return A pair of linear and angular speed in m/s and rad/s
     */
    std::pair<float, float> getTargetSpeed();

} // namespace UpLink
