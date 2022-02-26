#pragma once

#include "flightlib/objects/static_object.hpp"

class ArucoGate : public flightlib::StaticObject {
 public:
  ArucoGate(const std::string& id, const std::string& prefab_id = "aruco_gate")
    : flightlib::StaticObject(id, prefab_id) {}
  ~ArucoGate() {}
};