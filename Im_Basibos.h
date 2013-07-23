/**
 * @file Im_Basibos.h
 * @author Mustafa Özçelikörs
 * @brief Potansiyel alanlar kullanılarak ve ROS üzerinden sensör verilerini okuyarak Im_Basibosa ait Wander (Başıboş) davranışının gerçeklenmesidir. Başıboş gezinti sırasında Obstacle Avoidance (Engelden Kaçınma) davranışı da Im_Basibostan beklenmektedir. Sistem girdileri sensörlerin açıları, tüm sensörlere ait mesafe bilgileri, lineer ve açısal hız sabitleri olarak belirlenmiştir.
 */

#ifndef IM_BASIBOS_H_
#define IM_BASIBOS_H_

#include "Im_Eklenenler.h"

#define max(a,b) ((a)>(b)?(a):(b))
#define MAKSIMUM_UZAKLIK 2
#define ACISAL_HIZ_SABITI 2 //0.3 //0.7
#define LINEER_HIZ_SABITI 0.7 //0.5 //0.7
#define PI 3.14159265
#define SENSOR_ACISI_ON_KISIM 15
#define SENSOR_ACISI_YAN_KISIM 80 //60

double g_d_sensorlerin_okudugu_uzakliklar[12];


/***************************************************************************************
 *   Im_Basibos v1.1
 *   SINIF PROLOGU
 *
 * 	 AMAÇ:
 * 	 		Potansiyel alanlar kullanılarak ve ROS üzerinden sensör verilerini
 *   		okuyarak Im_Basibosa ait Wander (Başıboş) davranışının gerçeklenmesidir. Başıboş
 *   		gezinti sırasında Obstacle Avoidance (Engelden Kaçınma) davranışı da Im_Basibostan
 *   		beklenmektedir. Sistem girdileri sensörlerin açıları, tüm sensörlere ait mesafe
 *   		bilgileri, lineer ve açısal hız sabitleri olarak belirlenmiştir.
 *
 *
 *   Fonksiyon Prototipleri:
 *   		Im_Basibos();
 *    		void Im_SensorVerisiniAl(const sensor_msgs::LaserScan& okuma_degeri);
 *    		double Im_SensorVerisindenHamKuvvetDegeriAl (double d_sensor_girdisi);
 *    		vektor Im_IticiKuvvetiBul(double d_sensor_okuma_degeri,
 *		                        double d_arti_x_ile_yapilan_aci);
 *    		vektor Im_CekiciKuvvetiBul();
 *    		hiz Im_HizlariHesapla(vektor d_bileske_vektor_girdisi);
 *    		vektor Im_BileskeKuvvetiBul(double d_sensor_okuma_degerleri[12]);
 *    		hiz Im_BasibosDavranisi(double d_sensor_okuma_degerleri[12]);
 *			void Im_LogAl(float d_zaman,
 *						  konum k_mevcut_konum,
 *						  hiz h_uygulanan_hizlar);
 *
 *
 *
 *	 Global Değişkenler:
 *
 *    		bool g_b_sabit_acisal_hizla_calis;
 *    		double g_d_final_lineer_hiz
 *    		double g_d_final_acisal_hiz;
 *
 *
 *   Notlar:
 *   		Arka Sensörler Henüz Dahil Edilmemiştir.
 *
 *
 *   Yazar(lar):
 *
 *   		Mustafa Özçelikörs
 ****************************************************************************************/
class Im_Basibos
{
public:
     Im_Basibos();
     bool g_b_sabit_acisal_hizla_calis;
     double g_d_final_lineer_hiz, g_d_final_acisal_hiz;
     void Im_SensorVerisiniAl(const sensor_msgs::LaserScan& okuma_degeri);
     double Im_SensorVerisindenHamKuvvetDegeriAl (double d_sensor_girdisi);
     vektor Im_IticiKuvvetiBul(double d_sensor_okuma_degeri,
		                        double d_arti_x_ile_yapilan_aci);
     vektor Im_CekiciKuvvetiBul();
     hiz Im_HizlariHesapla(vektor d_bileske_vektor_girdisi);
     vektor Im_BileskeKuvvetiBul(double d_sensor_okuma_degerleri[12]);
     hiz Im_BasibosDavranisi(double d_sensor_okuma_degerleri[12]);
};

#endif /* IM_BASIBOS_H_ */
