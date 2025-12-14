#pragma once

#include <vector>
#include <cstdint>

namespace map_utils {

struct MapData {
    std::vector<uint8_t> pixels;
    int width{0};
    int height{0};

    [[nodiscard]] bool empty() const { return pixels.empty(); }

    [[nodiscard]] uint8_t at(int row, int col) const {
        return pixels[row * width + col];
    }
};

} // namespace map_utils
