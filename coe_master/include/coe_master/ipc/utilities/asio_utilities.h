#ifndef SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_IPC_UTILITIES_ASIO_UTILITIES
#define SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_IPC_UTILITIES_ASIO_UTILITIES

#include <boost/asio.hpp>
#include <iostream>

namespace coe_master
{

namespace utils
{
using Executor = std::function<void(const std::string& req, std::string& res)>;

class AsioSession : public std::enable_shared_from_this<AsioSession>
{
 public:
  AsioSession(boost::asio::ip::tcp::socket socket, Executor executor)
      : executor_(std::move(executor)), socket_(std::move(socket))
  {
  }

  void start() { do_read(); }

  Executor get_executor() { return executor_; };

 private:
  Executor executor_;
  void do_read()
  {
    auto self(shared_from_this());
    socket_.async_read_some(boost::asio::buffer(income_data_, max_length),
                            [this, self](boost::system::error_code ec, std::size_t /*length*/)
                            {
                              if (!ec)
                              {
                                std::string income{income_data_};
                                std::string outcome;
                                executor_(income, outcome);
                                short wlenght = std::min(int(outcome.size()), 1024);
                                std::memcpy(outcome_data_, outcome.data(), wlenght);
                                do_write(wlenght);
                              }
                            });
  }

  void do_write(std::size_t lenght)
  {
    auto self(shared_from_this());
    boost::asio::async_write(socket_,
                             boost::asio::buffer(outcome_data_, lenght),
                             [this, self](boost::system::error_code ec, std::size_t /*length*/)
                             {
                               if (!ec)
                               {
                                 do_read();
                               }
                             });
  }

  boost::asio::ip::tcp::socket socket_;
  enum : int
  {
    max_length = 1024
  };
  char income_data_[max_length];
  char outcome_data_[max_length];
};

class AsioServer
{
 public:
  using Ptr = std::shared_ptr<AsioServer>;
  using ConstPtr = std::shared_ptr<const AsioServer>;

  AsioServer(boost::asio::io_context& io_context, boost::asio::ip::tcp::endpoint ep, Executor executor)
      : executor_(executor), acceptor_(io_context, ep)
  {
    do_accept();
  }

 private:
  void do_accept()
  {
    acceptor_.async_accept(
        [this](boost::system::error_code ec, boost::asio::ip::tcp::socket socket)
        {
          if (!ec)
          {
            std::make_shared<AsioSession>(std::move(socket), executor_)->start();
          }
          do_accept();
        });
  }

  Executor executor_;
  boost::asio::ip::tcp::acceptor acceptor_;
};

using AsioServerPtr = AsioServer::Ptr;
using AsioServerConstPtr = AsioServer::ConstPtr;

inline AsioServerPtr make_shared(boost::asio::io_context& io_context,
                                 const std::string& host,
                                 short port,
                                 Executor executor)
{
  AsioServerPtr ret;
  auto ll = __LINE__;
  try
  {
    ll = __LINE__;
    boost::system::error_code ec;
    ll = __LINE__;
    boost::asio::ip::tcp::resolver resolver(io_context);
    ll = __LINE__;
    boost::asio::ip::tcp::resolver::iterator itEnd;
    ll = __LINE__;
    boost::asio::ip::tcp::resolver::query query(host, std::to_string(port));
    ll = __LINE__;
    boost::asio::ip::tcp::resolver::iterator it_endpoint = resolver.resolve(query, ec);
    ll = __LINE__;
    if (ec || it_endpoint == itEnd)
    {
      ll = __LINE__;
      std::stringstream ss;
      ss << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Could not resolve either machine (host: " << host
         << ", port: " << port << ")" << std::endl;
      throw std::runtime_error(ss.str().c_str());
    }

    ll = __LINE__;
    boost::asio::ip::tcp::endpoint ep =
        *it_endpoint;  // boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string( host ), port);
    ll = __LINE__;
    ret = AsioServerPtr(new AsioServer(io_context, ep, executor));
  }
  catch (const std::exception& e)
  {
    std::stringstream ss;
    ss << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Could not create a server (host: " << host << ", port: " << port
       << ")" << std::endl;
    ss << " Error: " << e.what() << std::endl;
    ss << " last Line executed: " << ll << std::endl;
    throw std::runtime_error(ss.str().c_str());
  }

  return ret;
}

class AsioClient
{
 public:
  void connect(const std::string& host, const std::string& service, std::chrono::steady_clock::duration timeout)
  {
    // Resolve the host name and service to a list of endpoints.
    auto endpoints = boost::asio::ip::tcp::resolver(io_context_).resolve(host, service);

    std::error_code error;
    boost::asio::async_connect(
        socket_,
        endpoints,
        [&](const std::error_code& result_error, const boost::asio::ip::tcp::endpoint& /*result_endpoint*/)
        { error = result_error; });

    // Run the operation until it completes, or until the timeout.
    run(timeout);

    // Determine whether a connection was successfully established.
    if (error) throw std::system_error(error);
  }

  std::string read_response(std::chrono::steady_clock::duration timeout)
  {
    std::error_code error;
    std::size_t n = 0;
    boost::asio::async_read_until(socket_,
                                  boost::asio::dynamic_buffer(input_buffer_),
                                  "\0",
                                  [&](const std::error_code& result_error, std::size_t result_n)
                                  {
                                    error = result_error;
                                    n = result_n;
                                  });

    // Run the operation until it completes, or until the timeout.
    run(timeout);

    // Determine whether the read completed successfully.
    if (error) throw std::system_error(error);

    std::string line(input_buffer_.substr(0, n - 6));
    input_buffer_.erase(0, n);
    return line;
  }

  std::error_code write_request(const std::string& line, std::chrono::steady_clock::duration timeout)
  {
    std::string data = line + "\n";

    std::error_code error;
    boost::asio::async_write(socket_,
                             boost::asio::buffer(data),
                             [&](const std::error_code& result_error, std::size_t /*result_n*/)
                             { error = result_error; });

    // Run the operation until it completes, or until the timeout.
    run(timeout);

    return error;
  }

 private:
  void run(std::chrono::steady_clock::duration timeout)
  {
    // Restart the io_context, as it may have been left in the "stopped" state
    // by a previous operation.
    io_context_.restart();

    // Block until the asynchronous operation has completed, or timed out. If
    // the pending asynchronous operation is a composed operation, the deadline
    // applies to the entire operation, rather than individual operations on
    // the socket.
    io_context_.run_for(timeout);

    // If the asynchronous operation completed successfully then the io_context
    // would have been stopped due to running out of work. If it was not
    // stopped, then the io_context::run_for call must have timed out.
    if (!io_context_.stopped())
    {
      // Close the socket to cancel the outstanding asynchronous operation.
      socket_.close();

      // Run the io_context again until the operation completes.
      io_context_.run();
    }
  }

  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_{io_context_};
  std::string input_buffer_;
};

}  // namespace utils
}  // namespace coe_master

#endif /* SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_IPC_UTILITIES_ASIO_UTILITIES */
