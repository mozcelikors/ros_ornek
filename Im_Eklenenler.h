/**
 * @file Im_Eklenenler.h
 * @author Mustafa Özçelikörs
 * @brief Global structları ve eklenen kütüphaneleri içeren header dosyasıdır
 */

#ifndef IM_EKLENENLER_H_
#define IM_EKLENENLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>


/**
 * @var typedef struct
 * @struct vektor
 */
typedef struct{
	  double d_vektorun_siddeti;
	  double d_aci_rad;
	  double d_x_eksenindeki_bileseni;
	  double d_y_eksenindeki_bileseni;
  } vektor;

/**
 * @var typedef struct
 * @struct hiz
 */
typedef struct{
	double d_dogrusal_hiz;
	double d_acisal_hiz;
} hiz;

/**
 * @var typedef struct
 * @struct konum
 */
typedef struct{
	double d_x_pozisyonu;
	double d_y_pozisyonu;
	double d_x_oryantasyonu;
	double d_y_oryantasyonu;
	double d_z_oryantasyonu;
	double d_w_oryantasyonu;
} konum;

#endif /* IM_EKLENENLER_H_ */
