#include <iostream>

#include "boost/asio/buffer.hpp"

#include "nlohmann/json.hpp"

#include "seat/solver.h"
#include "seat/train.h"

#include "net/stop_handler.h"
#include "net/web_server/responses.h"
#include "net/web_server/web_server.h"

using namespace net;
using namespace seat;
using net::web_server;
using json = nlohmann::json;

int main() {
  boost::asio::io_context ioc;
  web_server s{ioc};

  srand(0U);
  auto t = train();
  auto total_seats = 800;
  auto min_wagon_size = 50;
  auto max_wagon_size = 70;
  // min_wagon_size = max_wagon_size;
  t.generate_random_train(total_seats, 0.5, 0.3, 0.2, max_wagon_size,
                          min_wagon_size);
  t.print();
  std::map<reservation, uint32_t> a;

  auto const& gsd_prob = 0.0;
  auto const number_of_segments = 30U;
  auto seats = t.get_number_of_seats();

  auto fpb_solver = solver{seats, number_of_segments};

  s.on_http_request([&](web_server::http_req_t const& req,
                        web_server::http_res_cb_t const& cb, bool) {
    json::parse(req.body());
    return cb(server_error_response(req, "NOTHING TO SEE HERE - GO AWAY!"));
  });
  boost::system::error_code ec;
  s.init("127.0.0.1", "9000", ec);
  if (ec) {
    std::cout << "init error: " << ec.message() << "\n";
    return 1;
  }
  stop_handler const stop(ioc, [&]() {
    s.stop();
    ioc.stop();
  });
  std::cout << "web server running on http://127.0.0.1:9000/ and "
               "https://127.0.0.1:9000/\n";
  s.run();
  ioc.run();
}