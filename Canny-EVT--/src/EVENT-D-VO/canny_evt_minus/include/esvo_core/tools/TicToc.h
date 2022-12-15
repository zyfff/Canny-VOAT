#ifndef ESVO_CORE_TOOLS_TICTOC_H
#define ESVO_CORE_TOOLS_TICTOC_H

#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace esvo_core { namespace tools {
class TicToc {
public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000; // returns millisec
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
}}     // namespace esvo_core::tools
#endif // ESVO_CORE_TOOLS_TICTOC_H
