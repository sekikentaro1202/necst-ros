#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
from astropy.io import fits
import time
import threading



class logger(object):
    node_name = "Logger"
    
    # For callback, to fits 
    # ------------------------------
    init = {"OBJECT":"none", "OBSERVER":"none", "OBSMODE":"none", "MOLECULE":"none",
            "TRANSITE":"none", "LOFREQ":0, "SYNTH":0, "OBSNAME":"none"}
    
    weather = {"TAMBIENT":[], "HUMIDITYIN":[], "HUMIDITYOUT":[], "WINDSPEED":[],
            "WINDDIRECTION":[], "PRESSURE":[], "THOT1":[], "THOT2":[], "TIME":[]}
    
    encoder = {"AZIMUTH":[], "ELEVATIO":[], "TIME":[]}
    
    antenna = {"TARGETAZ":[], "TARGETEL":[], "TIME":[]}
    
    dome = {"STATUS":[], "RPOS":[], "LPOS":[], "MEMB":[], "REMSTATUS":[],
            "DOMEPOSITION":[], "TIME":[]}
    
    hot = {"SOBSMODE":[], "TIME":[]}
    
    m2 = {"SUBREF":[], "TIME":[]}


    hdu_prim = None
    hdu_1 = None
    hdu_2 = None
    hdu_3 = None


    def __init__(self):
        rospy.init_node(self.node_name)
        if __name__ == "_main_":
            thread = threading.Thread(target = self.)

    # Status not change during observation
    # ------------------------------------
    def log_once(self, req):
        self.init["OBJECT"] = req.OBJECT
        self.init["OBSERVER"] = req.OBSERVER
        self.init["OBSMODE"] = req.OBSMODE
        self.init["MOLECULE"] = req.MOLECULE
        self.init["TRANSITE"] = req.TRANSITE
        self.init["LOFREQ"] = req.LOFREQ
        self.init["SYNTH"] = req.SYNTH
        self.init["OBSNAME"] = req.OBSNAME
        
        header = fits.Header(self.init)
        self.hdu_prim = fits.PrimaryHDU(header = header)
        return

    # Get status function
    # -------------------
    def log_weather(self, req):
        self.weather["TAMBIENT"].append(req.out_temp)
        self.weather["HUMIDITYIN"].append(req.in_humi)
        self.weather["HUMIDITYOUT"].append(req.out_humi)
        self.weather["WINDSPEED"].append(req.wind_sp)
        self.weather["WINDDIRECTION"].append(req.wind_dir)
        self.weather["PRESSURE"].append(req.press)
        self.weather["THOT1"].append(req.cabin_temp1)
        self.weather["THOT2"].append(req.cabin_temp2)
        self.weather["TIME"].append(req.timestamp)
        return

    def log_encoder(self, req):
        self.encoder["AZIMUTH"].append(req.enc_az)
        self.encoder["ELEVATIO"].append(req.enc_el)
        self.encoder["TIME"].append(req.timestamp)
        return

    def log_antenna(self, req):
        self.antenna["TARGETAZ"].append(req.command_az)
        self.antenna["TARGETEL"].append(req.command_el)
        self.antenna["TIME"].append(req.timestamp)
        return

    def log_dome(self, req):
        self.dome["STATUS"].append(req.move_status)
        self.dome["RPOS"].append(req.right_pos)
        self.dome["LPOS"].append(req.left_pos)
        self.dome["MEMB"].append(req.memb_pos)
        self.dome["REMSTATUS"].append(req.remote_status)
        self.dome["DOMEPOSITION"].append(req.dome_enc)
        self.dome["TIME"].append(req.timestamp)
        return

    def log_hot(self, req):
        self.hot["SOBSMODE"].append(req.data)
        self.hot["TIME"].append(req.timestamp)
        return

    def log_m2(self, req):
        self.m2["SUBREF"].append(req.data)
        self.m2["TIME"].append(req.timestamp)
        return


    # Create Fits File 
    # ----------------
    def interval_1(self):
        """
        weather

        interval : 1 (s)
        """
        col1 = fits.Column(name='TAMBIENT', format='1D', unit='K', array=self.weather["TAMBIENT"])
        col2 = fits.Column(name='HUMIDITYIN', format='1D', unit='%', array=self.weather["HUMIDITYIN"])
        col3 = fits.Column(name='HUMIDITYOUT', format='1D', unit='%', array=self.weather["HUMIDITYOUT"])
        col4 = fits.Column(name='WINDSPEED', format='1D', unit='m/s', array=self.weather["WINDSPEED"])
        col5 = fits.Column(name='WINDDIRECTION', format='1D', unit='deg', array=self.weather["WINDDIRECTION"])
        col6 = fits.Column(name='PRESSURE', format='1D', unit='mm Hg', array=self.weather["PRESSURE"])
        col7 = fits.Column(name='THOT1', format='1D', unit='K', array=self.weather["THOT1"])
        col8 = fits.Column(name='THOT2', format='1D', unit='K', array=self.weather["THOT2"])
        col9 = fits.Column(name='TIME', format='1D', unit='s', array=self.weather["TIME"])
        coldefs = fits.ColDefs([col1, col2, col3, col4, col5, col6, col7, col8, col9])

        self.hdu_1 = fits.BinTableHDU.from_columns(coldefs)
        return

    def interval_01(self):
        """
        dome
        hot
        m2

        interval : 0.1 (s)
        """
        col1_d = fits.Column(name='DOMEPOSITION', format='1D', unit='deg', array=self.dome["DOMEPOSITION"])
        col2_d = fits.Column(name='TIME_DOME', format='1D', unit='s', array=self.dome["TIME"])
        col1_h = fits.Column(name='SOBSMODE', format='8A', array=self.hot["SOBSMODE"])
        col2_h = fits.Column(name='TIME_HOT', format='1D', unit='s', array=self.hot["TIME"])
        col1_m = fits.Column(name='SUBREF', format='1D', unit='', array=self.m2["SUBREF"])
        col2_m = fits.Column(name='TIME_M2', format='1D', unit='s', array=self.m2["TIME"])
        coldefs = fits.ColDefs([col1_d, col2_d, col1_h, col2_h, col1_m, col2_m])
        
        self.hdu_2 = fits.BinTableHDU.from_columns(coldefs)
        return

    def interval_001(self):
        """
        encoder
        antenna

        interval : 0.01 (s)
        """
        col1_e = fits.Column(name='AZIMUTH', format='1D', unit='deg', array=self.encoder["AZIMUTH"])
        col2_e = fits.Column(name='ELEVATIO', format='1D', unit='deg', array=self.encoder["ELEVATIO"])
        col3_e = fits.Column(name='TIME_ENC', format='1D', unit='s', array=self.encoder["TIME"])
        col1_a = fits.Column(name='TARGETAZ', format='1D', unit='deg', array=self.antenna["TARGETAZ"])
        col2_a = fits.Column(name='TARGETEL', format='1D', unit='deg', array=self.antenna["TARGETEL"])
        col3_a = fits.Column(name='TIME_ANT', format='1D', unit='s', array=self.antenna["TIME"])
        coldefs = fits.ColDefs([col1_e, col2_e, col3_e, col1_a, col2_a, col3_a])

        self.hdu_3 = fits.BinTableHDU.from_columns(coldefs)
        return

if __name__ == "__main__":
    log = logger()
    

    sub_status = rospy.Subscriber("log_status", Log_status_msg, log.log_once)
    sub_weather = rospy.Subscriber("status_weather", Status_weather_msg, log.log_weather)
    sub_enc = rospy.Subscriber("status_encoder", Status_encoder_msg, log.log_encoder)
    sub_antenna = rospy.Subscriber("status_antenna", Status_antenna_msg, log.log_antenna)
    sub_dome = rospy.Subscriber("status_dome", Status_dome_msg, log.log_dome)
    sub_hot = rospy.Subscriber("status_hot", String_necst, log.log_hot)
    sub_m2 = rospy.Subscriber("status_m2", Float64_necst, log.log_m2)
    rospy.spin()
