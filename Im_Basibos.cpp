/**
 * @file Im_Basibos.cpp
 * @author Mustafa Özçelikörs
 * @brief Im_Basibos sınıfına ait fonksiyonları içeren kaynak dosyasıdır.
 */

#include "Im_Basibos.h"

/***********************************************************************
 * FONKSIYON ADI: 				Im_Basibos::Im_Basibos
 * FONKSIYON AÇIKLAMASI: 		Im_Basibos sınıfının constructor fonksiyonudur.
 *
 * PARAMETRELER:
 * 			ADI				TIPI				AÇIKLAMASI
 *
 * DÖNÜS:
 * 			ADI				TIPI				AÇIKLAMASI
 *
 * GEREKLILIK:
 *
************************************************************************/
Im_Basibos::Im_Basibos()
{
    g_b_sabit_acisal_hizla_calis = false;
    g_d_final_lineer_hiz = 0;
    g_d_final_acisal_hiz = 0;
}


/***********************************************************************
 * FONKSIYON ADI: 				Im_Basibos::Im_SensorVerisindenHamKuvvetDegeriAl
 * FONKSIYON AÇIKLAMASI: 		Bu fonksiyon, sensörden gelen uzaklık
 *  girdisini alır, MAKSIMUM_UZAKLIK değişkenine bağlı bir fonksiyona
 *  tabii tutar ve kuvvetin büyüklük çıktısını çıktılar.
 * <!--
 * PARAMETRELER:
 * 			ADI				           TIPI				AÇIKLAMASI
 * 			d_sensor_girdisi             double           Sensörden gelen uzaklık bilgisi
 * DÖNÜS:
 * 			ADI				           TIPI				AÇIKLAMASI
 * 			d_kuvvet_buyukluk_ciktisi    double           Sensörden gelen uzaklık verisi kullanılarak
 * 			                                            fonksiyona tabi tutulan kuvvet değerinin çıktısı
 * -->
 * GEREKLILIK:
 *
************************************************************************/
/**
 * @param d_sensor_girdisi Sensörden gelen uzaklık bilgisi
 *
 * @return d_kuvvet_buyukluk_ciktisi Sensörden gelen uzaklık verisi kullanılarak fonksiyona tabi tutulan kuvvet değerinin çıktısı
 */
double Im_Basibos::Im_SensorVerisindenHamKuvvetDegeriAl (double d_sensor_girdisi)
{
	double d_kuvvet_buyukluk_ciktisi;
	if(d_sensor_girdisi < MAKSIMUM_UZAKLIK)
	{
		d_kuvvet_buyukluk_ciktisi = (MAKSIMUM_UZAKLIK - d_sensor_girdisi)/MAKSIMUM_UZAKLIK;
	}
	else
	{
		d_kuvvet_buyukluk_ciktisi = 0;
	}
	return d_kuvvet_buyukluk_ciktisi;
}




/***********************************************************************
 * FONKSIYON ADI: 				Im_Basibos::Im_BasibosDavranisi
 * FONKSIYON AÇIKLAMASI: 		Bu fonksiyon, diğer fonksiyonları çağırarak
 * davranışı gerçekleyecek hızı hesaplar ve hiz tipli olarak gönderir.
 * Girdi olarak sensörlerden okunan uzaklık değerini alır.
 * <!--
 * PARAMETRELER:
 * 			ADI				           TIPI				AÇIKLAMASI
 * 			d_sensor_girdisi             double           Sensörden gelen uzaklık bilgisi
 * DÖNÜS:
 * 			ADI				           TIPI				AÇIKLAMASI
 * 			d_kuvvet_buyukluk_ciktisi    double           Sensörden gelen uzaklık verisi kullanılarak
 * 			                                            fonksiyona tabi tutulan kuvvet değerinin çıktısı
 * -->
 * GEREKLILIK:
 * 			Öncesinde ros::init(), ros::NodeHandle, ros::Subscriber başlatılmalıdır.
 * 			ImSonar::Im_SensorBilgileriniAl geri çağırma fonksiyonundan gelen değer,
 * 			bu fonksiyona girdi olarak gönderilmelidir.
 *
************************************************************************/
/**
 * @param d_sensor_girdisi Sensörden gelen uzaklık bilgisi
 *
 * @return d_kuvvet_buyukluk_ciktisi Sensörden gelen uzaklık verisi kullanılarak fonksiyona tabi tutulan kuvvet değerinin çıktısı
 */
hiz  Im_Basibos::Im_BasibosDavranisi(double d_sensor_okuma_degerleri[12])
{
	vektor v_gecici_vektor_ciktisi = Im_Basibos::Im_BileskeKuvvetiBul(d_sensor_okuma_degerleri);
	hiz h_gecici_hiz_ciktisi = Im_Basibos::Im_HizlariHesapla(v_gecici_vektor_ciktisi);

		if ( !(h_gecici_hiz_ciktisi.d_acisal_hiz == 0) )
		{
			if ( (d_sensor_okuma_degerleri[0] < 1) && (d_sensor_okuma_degerleri[3] < 1) && ((d_sensor_okuma_degerleri[0]-d_sensor_okuma_degerleri[3]<0.3)||(d_sensor_okuma_degerleri[3]-d_sensor_okuma_degerleri[0]<0.3)) )
			{
				h_gecici_hiz_ciktisi.d_acisal_hiz = ACISAL_HIZ_SABITI;
			}
		}
	g_d_final_lineer_hiz = h_gecici_hiz_ciktisi.d_dogrusal_hiz;
	g_d_final_acisal_hiz = h_gecici_hiz_ciktisi.d_acisal_hiz;
	return h_gecici_hiz_ciktisi;
}

/***********************************************************************
 * FONKSIYON ADI: 				Im_Basibos::Im_IticiKuvvetiBul
 * FONKSIYON AÇIKLAMASI: 		Bu fonksiyon, Im_BileskeKuvvetiBul
 * fonksiyonunda tanimlanan itici kuvvetler için x ve y bileşenleri bulur.
 * Bu değerleri bulabilmek için sensörün uzaklık değerine ve tanımlanan
 * itici vektörün arti x ekseni ile yaptigi açıya ihtiyac duyar.
 *<!--
 * PARAMETRELER:
 * 			ADI			              	TIPI				AÇIKLAMASI
 * 			d_sensor_okuma_degeri         double              Sensörden okunan uzaklık değeri (metre)
 * 			d_arti_x_ile_yapilan_aci      double              Sensörün etkileyeceği kuvvetin x ekseni ile yaptığı açı (derece)
 * DÖNÜS:
 * 			ADI			             	TIPI				AÇIKLAMASI
 * 			v_itici_kuvvet_lokal          vektor              İtici kuvvetin x ve y bileşenleri
 * -->
 * GEREKLILIK:
 *
 *
************************************************************************/
/**
 * @param d_sensor_okuma_degeri Sensörden okunan uzaklık değeri (metre)
 * @param d_arti_x_ile_yapilan_aci Sensörün etkileyeceği kuvvetin x ekseni ile yaptığı açı (derece)
 *
 * @return v_itici_kuvvet_lokal İtici kuvvetin x ve y bileşenleri
 */
vektor Im_Basibos::Im_IticiKuvvetiBul(double d_sensor_okuma_degeri,
		                             double d_arti_x_ile_yapilan_aci)
{
	vektor v_itici_kuvvet_lokal;

	v_itici_kuvvet_lokal.d_vektorun_siddeti = Im_Basibos::Im_SensorVerisindenHamKuvvetDegeriAl(d_sensor_okuma_degeri);
	v_itici_kuvvet_lokal.d_aci_rad = d_arti_x_ile_yapilan_aci;
	v_itici_kuvvet_lokal.d_x_eksenindeki_bileseni = (v_itici_kuvvet_lokal.d_vektorun_siddeti) *  cos((d_arti_x_ile_yapilan_aci * PI)/180.0);
	v_itici_kuvvet_lokal.d_y_eksenindeki_bileseni = (v_itici_kuvvet_lokal.d_vektorun_siddeti) *  sin((d_arti_x_ile_yapilan_aci * PI)/180.0);

	return v_itici_kuvvet_lokal;
}


/***********************************************************************
 * FONKSIYON ADI: 				Im_Basibos::Im_CekiciKuvvetiBul
 * FONKSIYON AÇIKLAMASI: 		Bu fonksiyon, Im_BileskeKuvvetiBul
 * fonksiyonunda tanimlanan cekici kuvvetin x ve y bilesenlerini ciktilar.
 * <!--
 * PARAMETRELER:
 * 			ADI			           	TIPI				AÇIKLAMASI

 * DÖNÜS:
 * 			ADI				        TIPI				AÇIKLAMASI
 * 			v_cekici_kuvvet_lokal     vektor              Sanal hedef için oluşturulan çekici kuvvet
 * -->
 * GEREKLILIK:
 *
 *
************************************************************************/
/**
 * @return v_cekici_kuvvet_lokal Sanal hedef için oluşturulan çekici kuvvet
 */
vektor Im_Basibos::Im_CekiciKuvvetiBul()
{
	vektor v_cekici_kuvvet_lokal;

	v_cekici_kuvvet_lokal.d_x_eksenindeki_bileseni = 1;
	v_cekici_kuvvet_lokal.d_y_eksenindeki_bileseni = 0;

	return v_cekici_kuvvet_lokal;
}


/***********************************************************************
 * FONKSIYON ADI: 				Im_Basibos::Im_HizlariHesapla
 * FONKSIYON AÇIKLAMASI: 		Bu fonksiyon, Im_BileskeKuvvetiBul
 * fonksiyonunda bulunan bileske vektoru girdi olarak alır ve bu bileşke
 * vektörün x ve y bileşenlerinin ham büyüklüklerini sabit değerler ile
 * çarparak bir hiz vektörü çıktılar.
 *
 * <!--
 * PARAMETRELER:
 * 			ADI			               	TIPI				AÇIKLAMASI
 * 			v_bileske_vektor_girdisi      vektor              Bileşke Vektör Girdisi
 *
 * DÖNÜS:
 * 			ADI			             	TIPI				AÇIKLAMASI
 * 			h_sonuc_hizi                  hiz         Hız bileşenlerine çevirilen bileşke vektör
 * -->
 * GEREKLILIK:
 *
 *
************************************************************************/
/**
 * @param v_bileske_vektor_girdisi Bileşke vektör girdisi
 *
 * @return h_sonuc_hizi Hız bileşenlerine çevirilen bileşke vektör
 */
hiz Im_Basibos::Im_HizlariHesapla(vektor v_bileske_vektor_girdisi)
{
	hiz h_sonuc_hizi;

	//Önce bileske_kuvvet'un x ekseni ile yaptığı açıyı bulalım..
	double d_aci_rad_lokal = atan(v_bileske_vektor_girdisi.d_y_eksenindeki_bileseni/v_bileske_vektor_girdisi.d_x_eksenindeki_bileseni) * 180.0 / PI;
	double d_vektorun_siddeti_lokal = sqrt(pow(v_bileske_vektor_girdisi.d_x_eksenindeki_bileseni,2) + pow(v_bileske_vektor_girdisi.d_y_eksenindeki_bileseni,2));
	h_sonuc_hizi.d_dogrusal_hiz = LINEER_HIZ_SABITI * d_vektorun_siddeti_lokal * cos((d_aci_rad_lokal* PI)/180);
	h_sonuc_hizi.d_acisal_hiz = ACISAL_HIZ_SABITI * d_vektorun_siddeti_lokal * sin((d_aci_rad_lokal* PI)/180);

	return h_sonuc_hizi;
}

/***********************************************************************
 * FONKSIYON ADI: 				Im_Basibos::Im_BileskeKuvvetiBul
 * FONKSIYON AÇIKLAMASI: 		Bu fonksiyon Başıboş davranışında olan
 * kuvvetlerin Im_CekiciKuvvetiBul ve Im_IticiKuvvetiBul fonksiyonları
 * yardımıyla tanımlanmasını sağlar. Burada tanımlanan kuvvetlerin vektörel
 * bileşkesi, çıktı olarak verilir.
 * <!--
 * PARAMETRELER:
 * 			ADI			             	TIPI				AÇIKLAMASI
 * 			d_sensor_okuma_degerleri      double[12]          Sensör değerlerinden oluşan dizi
 *
 * DÖNÜS:
 * 			ADI			             	TIPI				AÇIKLAMASI
 * 			v_bileske_kuvvet              vektor              Bileşke Kuvvet
 * -->
 * GEREKLILIK:
 *
 *
************************************************************************/
/**
 * @param d_sensor_okuma_degerleri Sensör değerlerinden oluşan dizi
 *
 * @return v_bileske_kuvvet Bileşke kuvvet
 */
vektor Im_Basibos::Im_BileskeKuvvetiBul(double d_sensor_okuma_degerleri[12])
{
	vektor v_bileske_kuvvet;
	vektor v_cekici_kuvvet;
	vektor v_itici_kuvvet_sensor1; // Sensör 0'dan kaynaklanan repulsif kuvvet
	vektor v_itici_kuvvet_sensor2; // Sensör 3'den kaynaklanan repulsif kuvvet
	vektor v_itici_kuvvet_sensor3; // Sensör 4'den kaynaklanan repulsif kuvvet
	vektor v_itici_kuvvet_sensor4; // Sensör 11'den kaynaklanan repulsif kuvvet

	//Kuvvetler bu bölgede ekleniyor..
	v_itici_kuvvet_sensor1 = Im_Basibos::Im_IticiKuvvetiBul(d_sensor_okuma_degerleri[0], 180 + SENSOR_ACISI_ON_KISIM);
	v_itici_kuvvet_sensor2 = Im_Basibos::Im_IticiKuvvetiBul(d_sensor_okuma_degerleri[3], 180 - SENSOR_ACISI_ON_KISIM);
	v_itici_kuvvet_sensor3 = Im_Basibos::Im_IticiKuvvetiBul(d_sensor_okuma_degerleri[4], 180 + SENSOR_ACISI_YAN_KISIM);
	v_itici_kuvvet_sensor4 = Im_Basibos::Im_IticiKuvvetiBul(d_sensor_okuma_degerleri[11], 180 - SENSOR_ACISI_YAN_KISIM);
	v_cekici_kuvvet = Im_Basibos::Im_CekiciKuvvetiBul();

	//Bileşke kuvvet
	v_bileske_kuvvet.d_x_eksenindeki_bileseni = v_cekici_kuvvet.d_x_eksenindeki_bileseni +
								  v_itici_kuvvet_sensor1.d_x_eksenindeki_bileseni +
								  v_itici_kuvvet_sensor2.d_x_eksenindeki_bileseni +
								  v_itici_kuvvet_sensor3.d_x_eksenindeki_bileseni +
								  v_itici_kuvvet_sensor4.d_x_eksenindeki_bileseni;
	v_bileske_kuvvet.d_y_eksenindeki_bileseni = v_cekici_kuvvet.d_y_eksenindeki_bileseni +
								  v_itici_kuvvet_sensor1.d_y_eksenindeki_bileseni +
								  v_itici_kuvvet_sensor2.d_y_eksenindeki_bileseni +
								  v_itici_kuvvet_sensor3.d_y_eksenindeki_bileseni +
								  v_itici_kuvvet_sensor4.d_y_eksenindeki_bileseni;

	return v_bileske_kuvvet;
}


/***********************************************************************
 * FONKSIYON ADI: 				Im_Basibos::Im_SensorVerisiniAl
 * FONKSIYON AÇIKLAMASI: 		Sensör verisini ROS kütüphaneleri
 * yardımıyla alıp, hazırladığımız kuvvet ve hiz bulma fonksiyonlarina
 * gonderen fonksiyondur.
 * <!--
 * PARAMETRELER:
 * 			ADI				TIPI				            AÇIKLAMASI
 * 			okuma_degeri    const sensor_msgs::LaserScan&   ROS'un LaserScan kütüphanesinden gelen Sensör Bilgileri
 *
 * DÖNÜS:
 * 			ADI				TIPI				            AÇIKLAMASI
 * -->
 * GEREKLILIK:
 *
************************************************************************/
/**
 * @param okuma_degeri ROS'un LaserScan kütüphanesinden gelen sensör bilgileri
 */
void Im_Basibos::Im_SensorVerisiniAl(const sensor_msgs::LaserScan& okuma_degeri)
{
	double d_gecici_sensor_verileri[12] = {okuma_degeri.ranges[0],
									    okuma_degeri.ranges[1],
									    okuma_degeri.ranges[2],
									    okuma_degeri.ranges[3],
									    okuma_degeri.ranges[4],
									    okuma_degeri.ranges[5],
									    okuma_degeri.ranges[6],
									    okuma_degeri.ranges[7],
									    okuma_degeri.ranges[8],
									    okuma_degeri.ranges[9],
									    okuma_degeri.ranges[10],
									    okuma_degeri.ranges[11]};

	Im_Basibos::Im_BasibosDavranisi(d_gecici_sensor_verileri);
}
