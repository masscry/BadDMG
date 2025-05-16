#include "common.hh"

#include <deque>
#include <iostream>

namespace bdmg {

namespace {

constexpr std::size_t max_last_words = 10;

std::deque<std::string> last_words;

}

void add_last_words(std::string_view message) {
    last_words.emplace_back(message);
    if (last_words.size() > max_last_words) {
        last_words.pop_front();
    }
}

void panic(std::string_view message, const std::source_location location) noexcept {
    if (!last_words.empty()) {
        std::cerr << "LAST WORDS:" << std::endl;
    }
    for (const auto& line: last_words) {
        std::cerr << '\t' << line << "\n";
    }
    std::cerr << "PANIC at " << location.file_name() << ":" << location.line() << " in function "
              << location.function_name() << " - \n" << message << "\n";
    std::cerr.flush();
    std::abort();
}

}
