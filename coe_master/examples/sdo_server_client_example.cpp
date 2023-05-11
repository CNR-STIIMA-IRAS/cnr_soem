#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/writer.h>
#include <string>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <utility>

#include <coe_master/ipc/sdo_ipc.h>
#include <coe_master/ipc/sdo_ipc_msgs.h>
#include <coe_master/ipc/utilities/asio_utilities.h>

/**
 * @brief
 *
 */

struct ClientExample
{
  coe_master::utils::AsioClient c;

  ClientExample(const std::string& host, const std::string& port)
  {
    try
    {
      c.connect(host, port, std::chrono::seconds(10));
    }
    catch (std::exception& e)
    {
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Client Connection Error \n";
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Host: " << host << ", port: " << port << "\n";
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Exception: " << e.what() << "\n";
    }
  }

  template <typename R>
  void write_correct_request()
  {
    R req;
    req.index = 1;
    req.module_id = "elmo1";
    req.sdotype = req.TYPE_I16;
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
    req.sdotype = req.TYPE_I16;
    req.index = 0x1234;
    req.subindex = 1;
    std::string req_str = coe_master::to_string(req);
    c.write_request(req_str, std::chrono::seconds(10));
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
    std::cout << "Success: " << res.success << std::endl;
    return res.success;
  }
};

int main(int, char*[])
{
  pid_t c_pid = fork();
  std::shared_ptr<ClientExample> get_sdo_client;
  std::shared_ptr<ClientExample> set_sdo_client;

  constexpr unsigned short set_sdo_srv_port = 1024;
  constexpr unsigned short get_sdo_srv_port = 1025;

  if (c_pid == -1)
  {
    perror("fork");
    exit(EXIT_FAILURE);
  }
  else if (c_pid > 0)
  {
    std::map<int, std::string> module_addresses_map{{1, "elmo1"}, {2, "elmo2"}, {3, "elmo3"}};
    coe_master::SdoManager sdo_server(module_addresses_map, true, "localhost", set_sdo_srv_port, get_sdo_srv_port);

    usleep(20 * 1e6);
  }
  // else
  // {
  //   ClientExample ex("127.0.0.1", std::to_string(get_sdo_srv_port));

  //   ex.write_correct_request<coe_master::get_sdo_t::Request>();
  //   ex.read_response<coe_master::get_sdo_t::Response>();
  // }

  return 0;
}
