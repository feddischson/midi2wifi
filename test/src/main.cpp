// //////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2019 Christian Haettich [feddischson@gmail.com]
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include "gtest/gtest.h"

using MIDI_MSG = std::shared_ptr<std::vector<std::uint8_t> >;
using RNG_TYPE = boost::mt19937;

static constexpr int READ_MSG_TIMEOUT_MS = 100;

namespace {

class Midi_Generator {
  RNG_TYPE rng;
  boost::uniform_int<> rng_uniform;
  boost::variate_generator<RNG_TYPE, boost::uniform_int<> > dice;

 public:
  Midi_Generator()
      : rng(time(0)), rng_uniform(0, 255), dice(rng, rng_uniform) {}

  enum class Msg_Type : std::uint8_t {
    Note_Off = 0x80,
    Note_On = 0x90,
    Poly_Aftertouch = 0xA0,
    Control_Change = 0xB0,
    Program_Change = 0xC0,
    Mono_Aftertouch = 0xD0,
    Pitch_Bending = 0xE0,
    Sys_Exclusive = 0xF0,
    Time_Code = 0xf1,
    Song_Select = 0xf3,
    Song_Position_Counter = 0xf2,
    Undefined_1 = 0xf4,
    Undefined_2 = 0xf5,
    Tune_Request = 0xf6,
    Timing_Clock = 0xf8,
    Undefined_3 = 0xf9,
    Start = 0xfa,
    Continue = 0xfb,
    Stop = 0xfc,
    Undefined_4 = 0xfd,
    Active_Sync = 0xfe,
    System_Reset = 0xff
  };

  Msg_Type get_random_msg_type() {
    switch (dice() % 22) {
      case 0:
        return Msg_Type::Note_Off;
      case 1:
        return Msg_Type::Note_On;
      case 2:
        return Msg_Type::Poly_Aftertouch;
      case 3:
        return Msg_Type::Control_Change;
      case 4:
        return Msg_Type::Program_Change;
      case 5:
        return Msg_Type::Mono_Aftertouch;
      case 6:
        return Msg_Type::Pitch_Bending;
      case 7:
        return Msg_Type::Sys_Exclusive;
      case 8:
        return Msg_Type::Time_Code;
      case 9:
        return Msg_Type::Song_Select;
      case 10:
        return Msg_Type::Song_Position_Counter;
      case 11:
        return Msg_Type::Undefined_1;
      case 12:
        return Msg_Type::Undefined_2;
      case 13:
        return Msg_Type::Tune_Request;
      case 14:
        return Msg_Type::Timing_Clock;
      case 15:
        return Msg_Type::Undefined_3;
      case 16:
        return Msg_Type::Start;
      case 17:
        return Msg_Type::Continue;
      case 18:
        return Msg_Type::Stop;
      case 19:
        return Msg_Type::Undefined_4;
      case 20:
        return Msg_Type::Active_Sync;
      case 21:
        return Msg_Type::System_Reset;
      default:
        return Msg_Type::Undefined_1;
    }
  }

  std::shared_ptr<std::vector<std::uint8_t> > gen_message(Msg_Type type,
                                                          unsigned size = 0) {
    if (type == Msg_Type::Undefined_1 || type == Msg_Type::Undefined_2 ||
        type == Msg_Type::Tune_Request || type == Msg_Type::Timing_Clock ||
        type == Msg_Type::Undefined_3 || type == Msg_Type::Start ||
        type == Msg_Type::Continue || type == Msg_Type::Stop ||
        type == Msg_Type::Undefined_4 || type == Msg_Type::Active_Sync ||
        type == Msg_Type::System_Reset) {
      auto res = std::make_shared<std::vector<std::uint8_t> >(1);
      res->at(0) = static_cast<std::uint8_t>(type);
      return res;
    } else if (type == Msg_Type::Sys_Exclusive) {
      auto res = std::make_shared<std::vector<std::uint8_t> >(size);
      res->at(0) = 0xf0;
      res->at(size - 1) = 0xf7;
      for (unsigned idx = 0; idx < size - 2; idx++) {
        res->at(1 + idx) = dice() % 0x80;
      }
      return res;

    } else if (type == Msg_Type::Note_Off || type == Msg_Type::Note_On ||
               type == Msg_Type::Poly_Aftertouch ||
               type == Msg_Type::Control_Change ||
               type == Msg_Type::Pitch_Bending) {
      auto res = std::make_shared<std::vector<std::uint8_t> >(3);
      res->at(0) = static_cast<std::uint8_t>(type) | dice() & 0x0f;
      res->at(1) = dice() % 0x80;
      res->at(2) = dice() % 0x80;
      return res;
    } else if (type == Msg_Type::Program_Change ||
               type == Msg_Type::Mono_Aftertouch) {
      auto res = std::make_shared<std::vector<std::uint8_t> >(2);
      res->at(0) = static_cast<std::uint8_t>(type) | dice() & 0x0f;
      res->at(1) = dice() % 0x80;
      return res;
    } else if (type == Msg_Type::Time_Code || type == Msg_Type::Song_Select) {
      auto res = std::make_shared<std::vector<std::uint8_t> >(2);
      res->at(0) = static_cast<std::uint8_t>(type);
      res->at(1) = dice() % 0x80;
      return res;

    } else if (type == Msg_Type::Song_Position_Counter) {
      auto res = std::make_shared<std::vector<std::uint8_t> >(3);
      res->at(0) = static_cast<std::uint8_t>(type);
      res->at(1) = dice() % 0x80;
      res->at(2) = dice() % 0x80;
      return res;
    }
  }

  static void dump(MIDI_MSG& msg) {
    for (auto byte : *msg) {
      std::cout << std::hex << "0x" << std::setw(2) << std::setfill('0')
                << static_cast<unsigned>(byte) << " ";
    }
    std::cout << std::endl;
  }

};  // namespace

class M2W_Test : public ::testing::Test {
  boost::asio::io_service io;
  boost::asio::serial_port port;
  boost::asio::deadline_timer timer;
  bool read_error;
  size_t bytes_transferred;

  void read_complete(const boost::system::error_code& error,
                     size_t bytes_transferred) {
    // std::cout << "read complete" << std::endl;
    // std::cout << "error: " << error << ", transferred: " << bytes_transferred
    //          << std::endl;
    read_error = (error || bytes_transferred == 0);
    this->bytes_transferred = bytes_transferred;
    timer.cancel();
  }

  void time_out(const boost::system::error_code& error) {
    if (error) {
      return;
    }
    port.cancel();
  }

 protected:
  Midi_Generator midi_generator;
  M2W_Test() : port(io), timer(port.get_io_service()), read_error(false) {}

  virtual void SetUp() {
    port.open("/dev/ttyUSB0");
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
    auto res = read_with_timeout(128, 100);
  }

  virtual void TearDown() { port.close(); }

  void write(MIDI_MSG& msg) {
    boost::asio::write(port, boost::asio::buffer(*msg));
  }

  MIDI_MSG read_with_timeout(unsigned size, size_t timeout) {
    auto res = std::make_shared<std::vector<std::uint8_t> >(size);
    port.get_io_service().reset();

    // Initiate async. read
    boost::asio::async_read(
        port, boost::asio::buffer(*res),
        boost::bind(&M2W_Test::read_complete, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));

    // Setup timer to implement the read timeout.
    timer.expires_from_now(boost::posix_time::milliseconds(timeout));
    timer.async_wait(boost::bind(&M2W_Test::time_out, this,
                                 boost::asio::placeholders::error));

    // This will block until all characters are read
    // or until the it is cancelled.
    port.get_io_service().run();

    // std::cout << "transfered: " << bytes_transferred << std::endl;
    // for( auto byte : *res ) {
    //  std::cout << std::hex << static_cast<int>(byte )<< std::endl;
    //}

    if (bytes_transferred < size) {
      res->resize(bytes_transferred);
    }

    return res;
  }

  MIDI_MSG read(unsigned size) {
    auto res = std::make_shared<std::vector<std::uint8_t> >(size);
    boost::asio::read(port, boost::asio::buffer(*res));
    return res;
  }

  void perform_single_msg_test(Midi_Generator::Msg_Type type, size_t size = 0) {
    auto msg = midi_generator.gen_message(type, size);
    Midi_Generator::dump(msg);
    write(msg);
    auto res = read_with_timeout(msg->size(), READ_MSG_TIMEOUT_MS);
    EXPECT_EQ(*msg, *res);
  }

  double perform_timing_msg_test(size_t iterations) {
    std::vector<int> elapsed(iterations);
    std::array<int, 10> bins = {0};
    double elapsed_min = INT_MAX;
    double elapsed_max = 0;

    double mean = 0;
    for (size_t it = 0; it < iterations; ++it) {
      auto type = midi_generator.get_random_msg_type();
      auto msg = midi_generator.gen_message(type, 12);
      Midi_Generator::dump(msg);

      auto start = std::chrono::high_resolution_clock::now();
      write(msg);
      auto res = read_with_timeout(msg->size(), READ_MSG_TIMEOUT_MS);
      EXPECT_EQ(*msg, *res);
      auto stop = std::chrono::high_resolution_clock::now();
      int elapsed_useconds =
          std::chrono::duration_cast<std::chrono::microseconds>(stop - start)
              .count();

      // check for minimum
      if (elapsed_useconds < elapsed_min) {
        elapsed_min = elapsed_useconds;
      }

      // check for maximum
      if (elapsed_useconds > elapsed_max) {
        elapsed_max = elapsed_useconds;
      }
      elapsed.at(it) = elapsed_useconds;
      mean += elapsed_useconds;
    }

    mean = mean / iterations;

    std::cout << "min: " << elapsed_min << std::endl;
    std::cout << "max: " << elapsed_max << std::endl;
    std::cout << "mean: " << mean << std::endl;

    double d1 = mean - elapsed_min;
    double d2 = elapsed_max - mean;
    double size = elapsed_max - elapsed_min;
    double step = size / 10.0;

    for (auto val : elapsed) {
      if (val < (elapsed_min + step)) {
        ++bins.at(0);
      } else if (val > elapsed_max - step) {
        ++bins.at(9);
      } else {
        for (int i_bin = 1; i_bin < (10 - 1); ++i_bin) {
          double low = elapsed_min + i_bin * step;
          double high = low + step;
          if (val >= low && val < high) {
            ++bins.at(i_bin + 1);
            break;
          }
        }
      }
    }

    // get the maximum bin
    double max_bin = 0;
    for (auto bin : bins) {
      if (bin > max_bin) {
        max_bin = bin;
      }
    }

    std::cout << std::dec << "--- ";
    for (int i_bin = 0; i_bin < 10; ++i_bin) {
      double low = elapsed_min + i_bin * step;
      double high = low + step;
      double amount = bins.at(i_bin) / max_bin * 10.0;
      std::cout << low << " us" << std::endl << "   |";
      for (int i_amount = 0; i_amount < amount; ++i_amount) {
        std::cout << "+";
      }
      std::cout << " " << bins.at(i_bin) << std::endl;
      std::cout << "--- ";
    }
    std::cout << std::endl;
    return mean;
  }
};

TEST_F(M2W_Test, undefined_1_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Undefined_1, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, undefined_2_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Undefined_2, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, tune_request_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Tune_Request,
                            (i + 1) * 3);
  }
}

TEST_F(M2W_Test, timing_clock_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Timing_Clock,
                            (i + 1) * 3);
  }
}

TEST_F(M2W_Test, undefined_3_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Undefined_3, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, start_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Start, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, continue_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Continue, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, stop_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Stop, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, undefined_4_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Undefined_4, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, active_sync_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Active_Sync, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, system_reset_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::System_Reset,
                            (i + 1) * 3);
  }
}

TEST_F(M2W_Test, note_off_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Note_Off, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, note_on_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Note_On, (i + 1) * 3);
  }
}

TEST_F(M2W_Test, poly_after_touch_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Poly_Aftertouch);
  }
}

TEST_F(M2W_Test, control_change_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Control_Change);
  }
}

TEST_F(M2W_Test, program_change_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Program_Change);
  }
}

TEST_F(M2W_Test, mono_after_touch_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Mono_Aftertouch);
  }
}

TEST_F(M2W_Test, pitch_bending_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Pitch_Bending);
  }
}

TEST_F(M2W_Test, system_exclusive_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Sys_Exclusive,
                            (i + 1) * 3);
  }
}

TEST_F(M2W_Test, time_code_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Time_Code);
  }
}

TEST_F(M2W_Test, song_select_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Song_Select);
  }
}

TEST_F(M2W_Test, song_position_counter_message) {
  for (int i = 0; i < 10; i++) {
    perform_single_msg_test(Midi_Generator::Msg_Type::Song_Position_Counter);
  }
}

TEST_F(M2W_Test, timing) {
  double mean = perform_timing_msg_test(500);
  // the mean round-trip shall be less than 7 ms.
  EXPECT_LE(mean, 7000.0);
}

TEST_F(M2W_Test, large_block) {
  int size = 16;
  int n_msg = 100;

  auto msg_block = std::make_shared<std::vector<std::uint8_t> >();

  for (int i = 0; i < n_msg; i++) {
    auto msg =
        midi_generator.gen_message(midi_generator.get_random_msg_type(), size);
    if ((msg_block->size() + msg->size()) < 250) {
      msg_block->insert(msg_block->end(), msg->begin(), msg->end());
    }
  }
  std::cout << "Large block size: " << msg_block->size() << std::endl;
  write(msg_block);
  Midi_Generator::dump(msg_block);
  auto res = read_with_timeout(msg_block->size(), READ_MSG_TIMEOUT_MS);
  Midi_Generator::dump(res);
  EXPECT_EQ(*msg_block, *res);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// vim: filetype=cpp et ts=2 sw=2 sts=2
