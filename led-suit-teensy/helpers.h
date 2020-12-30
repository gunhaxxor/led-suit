#ifndef HELPERS_H
#define HELPERS_H

union binaryFloat {
  float f;
  unsigned char c[4];
};

union binaryInt16 {
  int16_t i;
  uint8_t c[2];
};

union anyType {
  float f;
  uint32_t ul;
  int32_t l;
  uint16_t ui[2];
  int16_t i[2];
  unsigned char c[4];
};

inline float toRadians(float x) { return x * 0.01745329252; }; // *pi/180
inline float toDegrees(float x) { return x * 57.2957795131; }; // *180/pi

inline void setHigh(int pin)
{
  digitalWrite(pin, HIGH);
}

inline void setLow(int pin)
{
  digitalWrite(pin, LOW);
}

inline void sendPulse(int pin)
{
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
}

inline void sendNegativePulse(int pin)
{
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);
}

inline uint8_t convertToByte(int value, int scaleBottom, int scaleTop)
{
  return map(constrain(value, min(scaleBottom, scaleTop), max(scaleBottom, scaleTop)), scaleBottom, scaleTop, 0, 255);
}

inline uint8_t maxByte(uint8_t a, uint8_t b) { return (a > b ? a : b); }

inline uint8_t minByte(uint8_t a, uint8_t b) { return (a < b ? a : b); }

inline int16_t floatToQ15(float value)
{
  return value * (0x01 << 15);
}

inline float Q15ToFloat(int16_t value)
{
  return ((float)value) / (0x01 << 15);
}

inline int numberOfSetBits(uint32_t i)
{
  // Java: use >>> instead of >>
  // C or C++: use uint32_t
  i = i - ((i >> 1) & 0x55555555);
  i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
  return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

//3d math
//quaternion is stored w, x, y, z
struct quaternion
{
  float w;
  float x;
  float y;
  float z;
};

struct vec3
{
  float x;
  float y;
  float z;
};

//creates a quaternion that first rotates q1 and then q2
inline quaternion quat_mult(const quaternion q1, const quaternion q2)
{
  quaternion qProd;
  qProd.w =
      (q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z);
  qProd.x =
      (q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y);
  qProd.y =
      (q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x);
  qProd.z =
      (q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w);
  return qProd;
};

inline quaternion quat_uniform_w(quaternion q)
{
  if (q.w < 0.f)
  {
    q.w = -q.w;
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
  }
  return q;
};

// Be aware. This function operates on the argument.
inline quaternion quat_norm(quaternion q)
{
  float normSF = 0;
  int i;
  float _q[4] = {q.w, q.x, q.y, q.z};
  for (i = 0; i < 4; i++)
  {
    normSF +=
        _q[i] * _q[i];
  }
  if (normSF > 0)
  {
    normSF = 1 / sqrt(normSF);
    for (i = 0; i < 4; i++)
    {
      _q[i] = _q[i] * normSF;
    }
  }
  else //bad input. Make a new unit quaternion.
  {
    _q[0] = 1.0f;
    for (i = 1; i < 3; i++)
    {
      _q[i] = 0;
    }
  }

  q.w = _q[0];
  q.x = _q[1];
  q.y = _q[2];
  q.z = _q[3];
  return q;
}

// If the quaternion is a unit quaternion then conjugate is same as inverse.
// Hence we can use the conjugate to reverse the rotation of unit quaternions.
// Be aware. This function operates on the argument.
inline quaternion quat_conj(quaternion q)
{
  // q[0] = -q[0];

  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  return q;
}

//rotates a vector using a quaternion
inline vec3 quat_rotate(quaternion q, vec3 vecIn)
{
  quaternion q_temp1, out4;
  // float q_temp1[4], q_in[4];
  // float in4[4], out4[4];

  // in4[0] = 0;
  quaternion in4 = {0.f, vecIn.x, vecIn.y, vecIn.z};
  // memcpy(&in4[1], vecIn, 3 * sizeof(float));
  // memcpy(&q_in, q, 4 * sizeof(float));
  q_temp1 = quat_mult(q, in4);
  q = quat_conj(q);
  out4 = quat_mult(q_temp1, q);
  // memcpy(vecOut, &out4[1], 3 * sizeof(float));
  return vec3{out4.x, out4.y, out4.z};
}

//calculates the relative rotation from q1 to q2
inline quaternion quat_delta_rotation(quaternion q1, quaternion q2)
{
  // float inv_q1[4];
  // quat_copy(q1, inv_q1);
  // float q2_copy[4];
  // quat_copy(q2, q2_copy);
  // quat_norm(inv_q1);
  // quat_conj(inv_q1);
  // quat_norm(q2_copy);
  // quat_mult(inv_q1, q2_copy, qOut);

  return quat_mult(quat_conj(quat_norm(q1)), quat_norm(q2));
}

inline void quat_build(const float *axis, float angle, float *qOut)
{
  qOut[0] = cos(angle / 2.0f);
  qOut[1] = axis[0] * sin(angle / 2.0f);
  qOut[2] = axis[1] * sin(angle / 2.0f);
  qOut[3] = axis[2] * sin(angle / 2.0f);
}

inline float quat_angle(quaternion q)
{

  //  Here's another formula taken from openFrameworks
  float sinhalfangle = sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  float result = 2.0 * atan2(sinhalfangle, q.w);
  //==================================================//

  // //Clamp value of w within -1 to 1
  // float w = q.w;
  // if (w > 1.0f)
  // {
  //   w = 1.0;
  // }
  // else if (w < -1.0f)
  // {
  //   w = -1.0;
  // }

  // float result = 2.0f * acos(w);
  //clamp to shortest angle (180 degrees)
  // if (result > PI)
  // {
  //   result -= 2.0f * PI;
  // }
  return result;
}

// calculates the angle between two quaternions
inline float quat_angle(const quaternion q1, const quaternion q2)
{
  return quat_angle(quat_uniform_w(quat_delta_rotation((q1), (q2))));
  // return quat_angle((quat_delta_rotation((q1), (q2))));
}

inline void quat_axis(const float *q, float *vecOut)
{
  float sinhalfangle = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  if (sinhalfangle)
  {
    vecOut[0] = q[1] / sinhalfangle;
    vecOut[1] = q[2] / sinhalfangle;
    vecOut[2] = q[3] / sinhalfangle;
  }
  else
  {
    vecOut[0] = 0.0;
    vecOut[1] = 0.0;
    vecOut[2] = 1.0;
  }
}

//yaw pitch roll. i.e. In the order they are applied to get the final rotation
inline void quat_eulerAngles(const float *q, float *e)
{
  e[0] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  e[1] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  e[2] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}

inline void vec_scale(float out[3], const float v[3], float scale)
{
  for (int c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale;
  }
}

inline void vec_add(float out[3], const float v1[3], const float v2[3])
{
  for (int c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}

inline void vec_copy(const float *v, float *vOut)
{
  memcpy(vOut, &v[0], 3 * sizeof(float));
}

inline float vec_dot(const float *v1, const float *v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline void vec_norm(float *v)
{
  float length = (float)sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (length > 0)
  {
    v[0] /= length;
    v[1] /= length;
    v[2] /= length;
    // return ofVec3f( x/length, y/length, z/length );
  }
  else
  {
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
  }
}

inline float vec_angle(const float *v1, const float *v2)
{
  float _vec1[3];
  float _vec2[3];
  vec_copy(v1, _vec1);
  vec_copy(v2, _vec2);
  vec_norm(_vec1);
  vec_norm(_vec2);
  return acos(vec_dot(_vec1, _vec2));

  // vec_norm(v1);
  // vec_norm(v2);
  // return acos(vec_dot(v1, v2));
}

inline float vec_length(const float *v)
{
  return (float)sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline float vec_lengthSquared(const vec3 v)
{
  return (float)(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline float vec_distance_squared(const float *v1, const float *v2)
{
  float vx = v1[0] - v2[0];
  float vy = v1[1] - v2[1];
  float vz = v1[2] - v2[2];
  return vx * vx + vy * vy + vz * vz;
}

inline float vec_distance(const float *v1, const float *v2)
{
  return (float)sqrt(vec_distance_squared(v1, v2));
}

#endif
