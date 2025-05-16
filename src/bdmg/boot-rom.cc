#include "boot-rom.hh"

#include <algorithm>
#include <vector>
#include <fstream>
#include <format>

namespace bdmg {

BootRom::BootRom(std::span<const u8, size> data, OnlySharedPtr) {
    std::copy(data.begin(), data.end(), _memory.begin());
}

SPBootRom BootRom::create(const fs::path& path) {
    if (path.empty()) {
        return SPBootRom{};
    }

    std::vector<u8> buffer;
    buffer.resize(size);
    auto file = std::ifstream(path, std::ios::binary);

    if (!file || file.bad()) {
        throw BootRomError(std::format("failed to open boot ROM file({})", path.native()));
    }

    if (fs::file_size(path) != size) {
        throw BootRomError("invalid boot ROM size");
    }

    file.read(reinterpret_cast<char*>(buffer.data()), size);
    return create_from_memory(std::span<const u8, size>(buffer));
}

SPBootRom BootRom::create_from_memory(std::span<const u8, size> buffer) {
    return std::make_shared<BootRom>(buffer, OnlySharedPtr{});
}


} // namespace bdmg
