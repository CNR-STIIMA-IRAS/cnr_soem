/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <map>

#include <cnr_ipc_utilities/asio_ipc.h>

#include <coe_master/ipc/sdo_ipc.h>
#include <coe_master/ipc/sdo_ipc_msgs.h>


auto now() { return std::chrono::steady_clock::now(); }

auto awake_time()
{
  using std::chrono::operator""ms;
  return now() + 2000ms;
}

namespace detail
{
struct unwrapper
{
  explicit unwrapper(std::exception_ptr pe) : pe_(pe) {}

  operator bool() const { return bool(pe_); }

  friend auto operator<<(std::ostream& os, unwrapper const& u) -> std::ostream&
  {
    try
    {
      std::rethrow_exception(u.pe_);
      return os << "no exception";
    }
    catch (std::runtime_error const& e)
    {
      return os << "runtime_error: " << e.what();
    }
    catch (std::logic_error const& e)
    {
      return os << "logic_error: " << e.what();
    }
    catch (std::exception const& e)
    {
      return os << "exception: " << e.what();
    }
    catch (...)
    {
      return os << "non-standard exception";
    }
  }
  std::exception_ptr pe_;
};
}  // namespace detail

auto unwrap(std::exception_ptr pe) { return detail::unwrapper(pe); }

template <class F>
::testing::AssertionResult does_not_throw(F&& f)
{
  try
  {
    f();
    return ::testing::AssertionSuccess();
  }
  catch (...)
  {
    return ::testing::AssertionFailure() << unwrap(std::current_exception());
  }
}

struct ClientExample
{
  cnr::ipc::AsioClient c;

  ClientExample(const std::string& host, const std::string& port)
  {
    try
    {
      c.connect(host, port, std::chrono::seconds(10));
    }
    catch (std::exception& e)
    {
      std::cerr << "Exception: " << e.what() << "\n";
    }
  }

  template <typename R>
  void write_correct_request()
  {
    R req;
    req.index = 1;
    req.module_id = "elmo1__1";
    req.sdotype = R::TYPE_I16;
    req.index = 0x1234;
    req.subindex = 1;
    std::string req_str = coe_master::to_string(req);
    std::error_code error = c.write_request(req_str, std::chrono::seconds(2));
    if (error)
    {
      std::cerr << error.message() << std::endl;
    }
  }

  template <typename R>
  void write_incorrect_request_1()
  {
    R req;
    req.index = 1;
    req.module_id = "ciao";
    req.sdotype = R::TYPE_I16;
    req.index = 0x1234;
    req.subindex = 1;
    std::string req_str = coe_master::to_string(req);
    std::error_code error = c.write_request(req_str, std::chrono::seconds(2));
    if (error)
    {
      std::cerr << error.message() << std::endl;
    }
  }

  void write_incorrect_request_2()
  {
    Json::Value write_root;
    write_root["what"] = "SET";
    write_root["address"] = 0x123456;
    Json::StreamWriterBuilder stream_write_builder;
    const std::string json_file = Json::writeString(stream_write_builder, write_root);
    c.write_request(json_file, std::chrono::seconds(10));
  }

  template <typename R>
  bool read_response()
  {
    std::string response = c.read_response(std::chrono::seconds(10));
    R res = coe_master::from_string<R>(response);
    if(!res.success)
      std::cout << "What: " << res.what << std::endl;
    return res.success;
  }
};

std::shared_ptr<ClientExample> get_sdo_client;
std::shared_ptr<ClientExample> set_sdo_client;
std::shared_ptr<coe_master::SdoManager> sdo_server;

constexpr unsigned short set_sdo_srv_port = 1024;
constexpr unsigned short get_sdo_srv_port = 1025;

TEST(TestSuite, srvCreation)
{
  std::map<int, std::string> module_addresses_map{{1, "elmo1"}, {2, "elmo2"}, {3, "elmo3"}};
  EXPECT_TRUE(does_not_throw(
      [&]
      {
        sdo_server.reset(
            new coe_master::SdoManager(module_addresses_map, true, "localhost", set_sdo_srv_port, get_sdo_srv_port));
      }));
}

TEST(TestSuite, clntSetSdoCreation)
{
  EXPECT_TRUE(
      does_not_throw([&] { set_sdo_client.reset(new ClientExample("localhost", std::to_string(set_sdo_srv_port))); }));
}

TEST(TestSuite, clntGetSdoCreation)
{
  EXPECT_TRUE(
      does_not_throw([&] { get_sdo_client.reset(new ClientExample("localhost", std::to_string(get_sdo_srv_port))); }));
}

TEST(TestSuite, clntSetSdo)
{
  EXPECT_TRUE(bool(sdo_server));
  EXPECT_TRUE(does_not_throw([&] { set_sdo_client->write_correct_request<coe_master::set_sdo_t::Request>(); }));
  EXPECT_TRUE(set_sdo_client->read_response<coe_master::set_sdo_t::Response>());
}

TEST(TestSuite, clntSetSdoErr1)
{
  EXPECT_TRUE(bool(sdo_server));
  std::cout << ">>>>>>>>>>>>>>>>>> SEND REQUEST" << std::endl;
  EXPECT_TRUE(does_not_throw([&] { set_sdo_client->write_incorrect_request_1<coe_master::set_sdo_t::Request>(); }));
  std::cout << ">>>>>>>>>>>>>>>>>> READ RESPONSE" << std::endl;
  EXPECT_FALSE(set_sdo_client->read_response<coe_master::set_sdo_t::Response>());
  std::cout << ">>>>>>>>>>>>>>>>>> DONE" << std::endl;
}

// TEST(TestSuite, clntSetSdoErr2)
// {
//   EXPECT_TRUE(bool(sdo_server));
//   EXPECT_TRUE(does_not_throw([&] { set_sdo_client->write_incorrect_request_2(); }));
//   EXPECT_FALSE(set_sdo_client->read_response<coe_master::set_sdo_t::Response>());
// }


// TEST(TestSuite, clntGetSdo)
// {
//   EXPECT_TRUE(sdo_server);
//   EXPECT_TRUE(does_not_throw([&] { get_sdo_client->write_correct_request<coe_master::get_sdo_t::Request>(); }));
//   EXPECT_TRUE(get_sdo_client->read_response<coe_master::get_sdo_t::Response>());
// }

// TEST(TestSuite, clntGetSdoErr)
// {
//   EXPECT_TRUE(does_not_throw([&] { get_sdo_client->write_incorrect_request_1<coe_master::set_sdo_t::Request>(); }));
//   EXPECT_FALSE(get_sdo_client->read_response<coe_master::get_sdo_t::Response>());
//   EXPECT_TRUE(does_not_throw([&] { get_sdo_client->write_incorrect_request_2(); }));
//   EXPECT_FALSE(get_sdo_client->read_response<coe_master::get_sdo_t::Response>());
// }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // try
  // {

  //   coe_master::SdoManager s;

  //   const auto start {now()};
  //   std::this_thread::sleep_until(awake_time());
  //   std::chrono::duration<double, std::milli> elapsed {now() - start};
  //   std::cout << "Waited " << elapsed.count() << " ms\n";
  // }
  // catch (std::exception& e)
  // {
  //   std::cerr << "Exception: " << e.what() << "\n";
  // }

  return RUN_ALL_TESTS();
}
