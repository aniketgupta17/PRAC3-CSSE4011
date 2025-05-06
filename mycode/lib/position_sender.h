#ifndef POSITION_SENDER_H
#define POSITION_SENDER_H

#ifdef __cplusplus
extern "C" {
#endif

void send_position_data(double x, double y, double vx, double vy);

#ifdef __cplusplus
}
#endif

#endif // POSITION_SENDER_H