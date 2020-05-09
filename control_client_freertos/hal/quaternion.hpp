// Copyright <2020> [Copyright rossihwang@gmail.com]
#pragma once

#include <cmath>
#include <cassert>

template <typename T>
class Quaternion {

public:
  T q1;
  T q2;
  T q3;
  T q4;

public:
  Quaternion(T q1=0, T q2=0, T q3=0, T q4=0)
    : q1(q1),
      q2(q2),
      q3(q3),
      q4(q4) {
    
  }
  Quaternion(const Quaternion<T> &q) {
    q1 = q.q1;
    q2 = q.q2;
    q3 = q.q3;
    q4 = q.q4;
  }
  T Norm() {
    return (q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); 
  }
  Quaternion<T> operator-(const Quaternion<T> q) {
    Quaternion<T> res;
    res.q1 = q1 - q.q1;
    res.q2 = q2 - q.q2;
    res.q3 = q3 - q.q3;
    res.q4 = q4 - q.q4;
    return res;
  }
  void operator-=(const Quaternion<T> q) {
    q1 -= q.q1;
    q2 -= q.q2;
    q3 -= q.q3;
    q4 -= q.q4;
  }
  Quaternion<T> operator*(const float k) {
    Quaternion<T> res;
    res.q1 = q1 * k;
    res.q2 = q2 * k;
    res.q3 = q3 * k;
    res.q4 = q4 * k;
    return res;
  }
  void operator*=(const float k) {
    q1 *= k;
    q2 *= k;
    q3 *= k;
    q4 *= k;
  }
  Quaternion<T> operator*(const Quaternion<T> &q) {
    Quaternion<T> res;
    res.q1 = q1 * q.q1 - q2 * q.q2 - q3 * q.q3 - q4 * q.q4;
    res.q2 = q1 * q.q2 + q2 * q.q1 + q3 * q.q4 - q4 * q.q3;
    res.q3 = q1 * q.q3 - q2 * q.q4 + q3 * q.q1 + q4 * q.q2;
    res.q4 = q1 * q.q4 + q2 * q.q3 - q3 * q.q2 + q4 * q.q1;
    return res;
  }
  void operator*=(const Quaternion<T> &q) {
    Quaternion<T> res;
    res.q1 = q1 * q.q1 - q2 * q.q2 - q3 * q.q3 - q4 * q.q4;
    res.q2 = q1 * q.q2 + q2 * q.q1 + q3 * q.q4 - q4 * q.q3;
    res.q3 = q1 * q.q3 - q2 * q.q4 + q3 * q.q1 + q4 * q.q2;
    res.q4 = q1 * q.q4 + q2 * q.q3 - q3 * q.q2 + q4 * q.q1;
    q1 = res->q1;
    q2 = res->q2;
    q3 = res->q3;
    q4 = res->q4;
  }
  Quaternion<T> operator/(const float k) {
    assert(k != 0);
    Quaternion<T> res;
    res.q1 = q1 / k;
    res.q2 = q2 / k;
    res.q3 = q3 / k;
    res.q4 = q4 / k;
    return res;
  }
  void operator/=(const float k) {
    // assert(k != 0);
    q1 /= (k + 10e-6);
    q2 /= (k + 10e-6);
    q3 /= (k + 10e-6);
    q4 /= (k + 10e-6);
  }
};
