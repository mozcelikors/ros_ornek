#include "Im_RobotKontrol.h"


int main(int argc, char **argv)
{

	/**** Konum ve Zaman Deneme 
	Im_Konum im_Konum;
	printf("%d",im_Konum.Im_DakikaAl());
	***/


	/****
	 * XML Ayıklayıcı Deneme
	// Eğer undefined reference to Class:Class hatası alınırsa, boş yere constructor tanımlamadan kaynaklı olabilir.
	Im_XMLAyiklayici im_XMLAyiklayici;
	im_XMLAyiklayici.Im_DinamikKinematikParametrelerAyikla();
	printf("%lf\n",im_XMLAyiklayici.KinematikMotorGetir().d_dogrusal_arzu_edilen_hiz);
	return 1;
	***/




	ros::init(argc, argv, "listener_class");
	ros::NodeHandle n;

	konum k_hedefin_konumu;
	hiz h_max_hiz_degerleri;
	hiz h_uygulanacak_hizlar;
	double d_varis_hata_toleransi;

	// Hedefe Gitme Davranışı için değişkenler initialize ediliyor.
	k_hedefin_konumu.d_x_pozisyonu = 25.0; // m
	k_hedefin_konumu.d_y_pozisyonu = 25.0; // m

	h_max_hiz_degerleri.d_dogrusal_hiz = 0.5; // m/s
	h_max_hiz_degerleri.d_acisal_hiz = 0.3; // rad/s
	d_varis_hata_toleransi = 1.0; //m

	//int DAVRANIS=0;


	//Objeler üretelim
	//Im_Sonar im_Sonar;
	//Im_Konum im_Konum;
	//Im_HedefeGitme im_HedefeGitme;
	Im_Basibos im_Basibos;


	// Subscriber ve Publisher ların 2. argümanı veri alan buffer büyüklüğünü ayarlamada kullanılıyor.
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/gz/cmd_vel", 1000);
	// Sonar verisini al
	ros::Subscriber sub = n.subscribe("/gazebo/scan", 1000, &Im_Basibos::Im_SensorVerisiniAl, &im_Basibos);

	//Konum verisini al
	//ros::Subscriber sub_pose = n.subscribe("/gz/odom", 1000, &Im_Konum::Im_PozisyonAl, &im_Konum);

	// Subscriber ve Publisher'dan veri alımı ve gönderimi işleminin frekansı ayarlanıyor.
	ros::Rate loop_rate(2);

	// Gönderilecek hız değerleri için geometry_msgs::Twist sınıfından gönderilecek_hizlar adından
	// nesne oluşturuluyor.
	geometry_msgs::Twist gonderilecek_hizlar;

	// ros::ok() komutu Ctrl-C tuşuna basılıp basılmadığını kontrol ediyor. Basıldı ise sonuç 0 oluyor.
	// Böylece kodu kapatma işlemi terminalde gerçekleşmiş oluyor.
	while(ros::ok())
	{

		switch(DAVRANIS)
		{
			case 0:// HedefeGit
				//h_uygulanacak_hizlar = im_HedefeGitme.Im_HedefeGitmeDavranisi(g_k_aracin_konumu,
																			  k_hedefin_konumu,
																			  h_max_hiz_degerleri,
																			  d_varis_hata_toleransi);
				break;
			case 1: //Basibos
				h_uygulanacak_hizlar = im_Basibos.Im_BasibosDavranisi(g_d_sensorlerin_okudugu_uzakliklar);
				break;
		}



		gonderilecek_hizlar.linear.x = h_uygulanacak_hizlar.d_dogrusal_hiz;
		gonderilecek_hizlar.linear.y = 0.0;
		gonderilecek_hizlar.linear.z = 0.0;

		gonderilecek_hizlar.angular.x = 0.0;
		gonderilecek_hizlar.angular.y = 0.0;
		gonderilecek_hizlar.angular.z = h_uygulanacak_hizlar.d_acisal_hiz;

		chatter_pub.publish(gonderilecek_hizlar);
    	ros::spinOnce();
  }
  return 0;
  *****/
}
