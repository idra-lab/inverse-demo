#ifndef CLIENT_PUBLISHER_HPP
#define CLIENT_PUBLISHER_HPP

#include <sl/Camera.hpp>

class ClientPublisher {
public:
  sl::Camera zed;

  ClientPublisher();
  ~ClientPublisher();

  bool open(sl::InputType input,
            sl::COORDINATE_SYSTEM coord_system,
            sl::RESOLUTION resolution,
            int sdk_gpu_id);

  bool grab();   // 🔴 NEW: explicit grab
  void close();
};

#endif