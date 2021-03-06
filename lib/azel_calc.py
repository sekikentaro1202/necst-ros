import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
from astropy.time import Time
from datetime import datetime as dt
import time
import math
import sys
sys.path.append('/home/amigos/ros/src/necst/lib')
sys.path.append('/home/necst/ros/src/necst/lib')
import coord
import numpy as np
import func_calc

class azel_calc(object):
    
    latitude = -22.96995611
    longitude = -67.70308139
    height = 4863.85
    utc_offset = 0.# utc_time
    soft_limit = 240.

    vel_dt = 0.1
    off_az = 0.
    off_el = 0.
    loop_rate = 0.1

    planet = {1:"mercury", 2:"venus", 3:"earth", 4:"mars", 5:"jupiter", 6:"saturn", 7:"uranus", 8:"neptune", 10:"moon", 11:"sun"}

    def __init__(self):
        self.coord = coord.coord_calc()
        self.first_time = time.time()
        pass


    def dcos_calc(self, x, y, off_x, off_y, dcos):
        if dcos == 1:
            y += off_y
            x += off_x/np.cos(np.radians(y))            
        else:
            y += off_y
            x += off_x
        return [x, y]
        
    def kisa_calc(self, altaz, dcos, hosei):
        ret_azel = self.dcos_calc(altaz.az.arcsec, altaz.alt.arcsec, self.off_az, self.off_el, dcos)
        #ret_azel = self.dcos_calc(40, 50, self.off_az, self.off_el, dcos)
        #ret_azel = [60,30]
        #print(str(num))
        _az = np.radians(ret_azel[0]/3600.)
        _el = np.radians(ret_azel[1]/3600.)
        ret = self.coord.apply_kisa_test(_az, _el, hosei)
        target_az = ret_azel[0]+ret[0]
        target_el = ret_azel[1]+ret[1]
        return target_az, target_el

    def azel_calc(self, az, el, off_x, off_y, off_coord, now, func_x=0, func_y=0, movetime=10):
        if off_coord.lower() != "horizontal":
            print("Please, off_coord is HORIZONTAL")
            return
        else:
            pass
        tv = time.time()
        param_x = param_y = lambda x:0
        """
        if func_x:
            func_x.append([0]*(4-len(func_x)))
            param_x = func_calc.calc(func_x[0],func_x[1], func_x[2], func_x[3])
        else:
            param_x = lambda x : 0
            pass
        if func_y:
            func_y.append([0]*(4-len(func_y)))
            param_y = func_calc.calc(func_y[0],func_y[1], func_y[2], func_y[3])
        else:
            param_y = lambda x : 0
            pass
        """
        az_list = [az*3600.+ off_x + param_x(x*0.1) for x in range(int(movetime*10))]
        el_list = [el*3600.+ off_y + param_y(y*0.1) for y in range(int(movetime*10))]
        print("az : ", az_list[0],"el : ",el_list[0] )
        return [az_list, el_list, tv]

    def coordinate_calc(self, x, y, coord, ntarg, off_x, off_y, offcoord, hosei, lamda, dcos, temp, press, humi, now, movetime=10, limit=True):
        #print("parameter : ", x, y, ntarg, coord, off_x, off_y, offcoord, hosei, lamda, dcos, temp, press, humi, now, movetime)
        #print("site position(latitude,longitude) : ", (self.latitude*u.deg, self.longitude*u.deg))

        # coordinate check
        if coord.lower() == "j2000":
            on_coord = SkyCoord(x, y,frame='fk5', unit='deg',)
        elif coord.lower() =="b1950":
            on_coord = SkyCoord(x, y, frame='fk4', unit='deg',)
        elif coord.lower() =="galactic":
            on_coord = SkyCoord(x, y, frame='galactic', unit='deg',)
        elif coord.lower() =="planet":
            print("planet_move")
        else:
            print("coord error !!")
            sys.exit()


        if offcoord.lower() == "j2000" or offcoord.lower() == "equatorial":
            off_coord = SkyCoord(off_x,off_y,frame='fk5',unit='arcsec',)
            ret = self.dcos_calc(on_coord.fk5.ra.deg, on_coord.fk5.dec.deg, 
                      off_coord.fk5.ra.deg, off_coord.fk5.dec.deg, dcos)
            real_coord = SkyCoord(ret[0], ret[1], frame='fk5', unit='deg',)

        elif offcoord.lower() == "b1950":
            off_coord = SkyCoord(off_x,off_y,frame='fk4',unit='arcsec',)
            ret = self.dcos_calc(on_coord.fk4.ra.deg, on_coord.fk4.dec.deg, 
                            off_coord.fk4.ra.deg, off_coord.fk4.dec.deg, dcos)
            real_coord = SkyCoord(ret[0], ret[1], frame='fk4', unit='deg',)

        elif offcoord.lower() == "galactic":
            off_coord = SkyCoord(off_x,off_y,frame='galactic',unit='arcsec',)
            ret = self.dcos_calc(on_coord.galactic.l.deg, on_coord.galactic.b.deg, 
                            off_coord.galactic.l.deg, off_coord.galactic.b.deg, dcos)
            real_coord = SkyCoord(ret[0], ret[1], frame='galactic', unit='deg',)

        elif offcoord.lower() == "horizontal":
            self.off_az = off_x #arcsec
            self.off_el = off_y #arcsec
            if not coord.lower() =="planet":
                real_coord = on_coord
            else:
                pass
        else:
            print("############## no offcoord ##############")
            pass
        
        # convert_azel
        nanten2 = EarthLocation(lat = self.latitude*u.deg, lon = self.longitude*u.deg, height = self.height*u.m)
        if int(movetime*100) == 1:
            list_num = int(len(x))
            print("###otf_mode###")
        else:
            list_num = int(movetime*10)
            
        #tstr = now.strftime('%Y-%m-%d %H:%M:%S')
        time_list = [Time(now)-self.utc_offset*u.hour+(i*self.loop_rate)*u.s for i in range(list_num)]
        if coord.lower() =="planet":
            real_coord = get_body(self.planet[ntarg], Time(now))
        else:
            pass
        real_coord.location=nanten2
        real_coord.pressure=press*u.Pa#param
        real_coord.temperature=temp*u.deg_C#param
        real_coord.relative_humidity=humi#param
        real_coord.obswl = lamda*u.um#param
        altaz = real_coord.transform_to(AltAz(obstime=time_list))

        print("create_list : start!!")
        altaz_list = [altaz[i] for i in range(list_num)]# shorting calc time
        #az_list = [self.kisa_calc(altaz_list[i], dcos, hosei)[0] for i in range(int(movetime*10))]
        #el_list = [self.kisa_calc(altaz_list[i], dcos, hosei)[1] for i in range(int(movetime*10))]
        az_list=[]
        el_list=[]
        for i in range(list_num):
            azel_list = self.kisa_calc(altaz_list[i], dcos, hosei)
            az_list.append(azel_list[0])
            el_list.append(azel_list[1])
        if limit == True:
            check_list1 = list(filter(lambda x:0.<x<240.*3600., az_list))
            check_list2 = list(filter(lambda x:120.*3600.<x<360.*3600., az_list))
        elif limit == False:
            check_list1 = list(filter(lambda x:0.<x<270.*3600., az_list))
            check_list2 = list(filter(lambda x:90.*3600.<x<360.*3600., az_list))
        else:
            print("!!!!!limit set error!!!!!")

        if  check_list1 == az_list:
            pass
        elif check_list2 == az_list:
            az_list = list(map(lambda x:x-360.*3600.,az_list))
        else:
            print("warning : range is over 270 [deg] !!")
            pass
        #print("az list!! : ", az_list)
        print("create_list : end!!")
        print("#######time#######",str(time.time()-self.first_time))
        now = float(now.strftime("%s")) + float(now.strftime("%f"))*1e-6#utc
        print("az :",az_list[0]/3600.,"el :", el_list[0]/3600., "time : ", now)
        #self.test2 = time.time()
        #print("!!!!!time!!!!!", self.test2-self.test1)
        return[az_list, el_list, now]
            

if __name__ == "__main__":
    qq = azel_calc()
    from datetime import datetime as dt
    now = dt.utcnow()
    #qq.coordinate_calc([30,23,23]*10, [40,23,34]*10, "j2000", 7, off_x=10, off_y=10, offcoord="horizontal", hosei="hosei_230.txt", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, movetime = 0.01)
    qq.azel_calc(0,45,0,0,"horizontal", now, ["a*cos(x)",100,10,5],movetime=10)
    qq.coordinate_calc([229,230,231], [-29,-30,-31], "j2000", 7, off_x=10, off_y=10, offcoord="horizontal", hosei="hosei_230.txt", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, movetime = 0.01,limit=False)
    
    
