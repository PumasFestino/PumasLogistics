#!/usr/bin/env python
import rospy
import random
import numpy as np
import datetime
from std_msgs.msg import String


def ordergen():
    #Order 1: 1 x C0 from 00:00 to 17:00
    order_arr = np.zeros([10,3])
    for i in range(9):
        order_num = i
        order_cmplx = random.randint(1,2)
        order_type = random.randint(0,3)
        order_arr[i,0] = order_num+1
        order_arr[i,1] = order_cmplx
        order_arr[i,2] = order_type
    return order_arr

def mps_state_gen():
    global rate
    machine_types = ['CS','RS','DS','SS','BS']
    mps_type = machine_types[random.randint(0,len(machine_types))]
    mps_num = random.randint(1,2)
    fail_time = random.randint(0,10)
    mps_names = ['M-'+mps_type+str(int(mps_num)),'C-'+mps_type+str(int(mps_num))] 
    state_msg = 'Machine '+mps_names[0]+' down for '+str(int(fail_time))+' sec \nMachine '+mps_names[1]+' down for '+str(int(fail_time))+' sec'
    print(state_msg)
    rospy.sleep(fail_time)
    state_msg = 'Machine '+mps_names[0]+' is up again \nMachine '+mps_names[1]+' is up again'
    print(state_msg)

def zone_msg_pub(pub, zone_msg):
    print(zone_msg)
    pub.publish(zone_msg)

def rb_msg_pub(pub, rb_msg):
    print(rb_msg)
    pub.publish(rb_msg)
    

def main():
    rospy.init_node('fake_refbox', anonymous=True)
    global rate
    rate = rospy.Rate(5)

    pub = rospy.Publisher('/refbox_msg', String, queue_size=10)
    pub_zone = rospy.Publisher('/zone_msg', String, queue_size=10)

    used_zones = np.array([])
    used_zones_indx = np.array([])
    used_indx_order = np.array([])

    order_arr = ordergen()
    #mps_state_gen()

    #zone_strings = ['M_Z64 ','M_Z63 ','M_Z62 ','M_Z61 ','M_Z53 ','M_Z43 ','M_Z34 ','M_Z24 ','M_Z15 ','C_Z75 ','C_Z65 ','C_Z54 ','C_Z45 ','C_Z44 ','C_Z35 ','C_Z34 ']

    #Del arreglo zone_strings se toman 12 zonas de manera aleatoria (Descomentar si se necesita)
    """zone_strings = ['M_Z11 ','M_Z12 ','M_Z13 ','M_Z14 ','M_Z15 ','M_Z21 ','M_Z22 ','M_Z23 ','M_Z24 ','M_Z25 ','M_Z31 ','M_Z32 ','M_Z33 ','M_Z34 ','M_Z35 ','M_Z41 ', 'M_Z42 ', 'M_Z43 ', 'M_Z44 ','M_Z45 ', 'M_Z51 ','M_Z52 ','M_Z53 ','M_Z54 ','M_Z55 ']
    rnd_zones = np.random.choice(zone_strings, 12, replace = False)
    print("".join(rnd_zones))"""

    #Arreglo con zonas elegidas disponibles dentro del lab (logisticsZones)
    #Zonas en orden de entrada al lab hasta el fondo
    #zone_strings = ['M_Z41 ','M_Z22 ','M_Z13 ','C_Z23 ','C_Z32 ','C_Z42']
    #Zonas en desorden
    zone_strings = ['C_Z32 ','M_Z22 ','C_Z42 ','C_Z23 ','M_Z41 ','M_Z13']
    

    while not rospy.is_shutdown():
        #Descomentar cuando se use el arreglo de zonas aleatorias
        #zone_msg_pub(pub_zone, "".join(rnd_zones))

        #Descomentar cuando se use directamente zone_strings
        zone_msg_pub(pub_zone, "".join(zone_strings))

        rate.sleep()
        
	    
        """while used_zones.shape[0] < 12 and not rospy.is_shutdown():
            zone_indx = random.randint(0,len(zone_strings)-1)
            if not (zone_indx in used_zones_indx) and zone_indx != -1:
                used_zones_indx = np.append(used_zones_indx, [int(zone_indx)])
                used_zones = np.append(used_zones, zone_strings[zone_indx])
                #zone = zone_strings[zone_indx]
            if(used_zones.shape[0] == 12):
                zone_msg_pub(pub_zone, used_zones)
                rate.sleep()
        rate.sleep()"""
            
        '''
        order_indx = random.randint(0,order_arr.shape[0]) -1
        if not (order_indx in used_indx_order) and order_indx != -1:
            order_msg = 'Order '+str(int(order_arr[order_indx,0]))+': '+str(int(order_arr[order_indx,1]))+' x C'+str(int(order_arr[order_indx,2]))
            used_indx_order = np.append(used_indx_order, [int(order_indx)])
            rb_msg_pub(pub, order_msg)
            if used_indx_order.shape[0] >= 10:
                print("All orders published")'''
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
