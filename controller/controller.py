import Sofa
import os
import math

class softLayerController(Sofa.PythonScriptController):

    # optionnally, script can create a graph...     
    def initGraph(self,node):
        integration = node.getChild('Integration')
        self.node1 = integration.getChild('bellow_cav')
        self.engine = integration.getObject('bellow_obj')
        self.cavity = self.node1.getObject('SurfacePressureConstraint')
        self.box3 = integration.getObject('boxROI3')
        self.box1 = integration.getObject('boxROI1')
        self.constraint = integration.getObject('LayerConstraint1')
        self.time = 0
        self.p = 0
        return 0

    def onBeginAnimationStep(self,dt):
        indice1 = self.box1.findData('indices').value
        indice3 = self.box3.findData('indices').value
        
        self.time = self.time + dt
        self.p = self.p + dt
        if self.time < 3:
            self.constraint.findData('indices').value = indice3
            self.p = 0
        else:
            self.cavity.findData('value').value = self.p

        return 0






############################################################
#        print(x)

        #for i in xrange(0,len(x)):
        #    a = x[i][1]
        #    a -= 1
        #    x[i][1] = a


#        print(self.time)
############################################################
#        translation = self.engine2.translation
#        forces = self.engine2.force
		## translation speed
 #       speed = 1  
 #       force = -200
        
#        translation[0][1] += 1

        #for i in xrange(0,len(forces)):
        #    print(i)
        #    forces[i][1] = 50
            #if position[20][1] == 10:
            #    #self.engine.velocity = self.engine.reset_velocity
            #    break

           #index = x[0]
           #forces[index][1] = force
#        self.engine2.translation = translation


#############################################################   Controller
        #gain_P = -100
        #gain_I = -30
        #gain_D = -30

        #position1 = self.engine.position[172][1]
        #reference = 28
        
        #error = Error(position1,reference)
        
        #self.inte += dt*error
        #self.prev_error = error
        #pressure = Pcontroller(error,gain_P) + Icontroller(self.inte,dt,gain_I) + Dcontroller(error, dt, gain_D)
        #if pressure > 700:
        #    pressure = 700
        #if pressure < -700:
        #    pressure = -700

        #self.cavity.findData('pressure').value = pressure
        #print(position1)
        #print(pressure)
        #position = self.engine2.findData('input_position').value 
        #position[0][1] += 1
        #self.engine2.findData('input_position').value = position
        #print(position[0][1])
#################################################
# def onBeginAnimationStep(self,dt):

     # adding at each time step a rotation angle in z (Euler angle) of 50 degree
     #PosZ = self.object.findData('translation').value[0][2]
#     PosZ = self.object.position
 #    PosZ[0][2]  = PosZ[0][2]  - 5
 #    print PosZ[0][2]
 #    self.object.position= PosZ[0][2]
 #    return 0
 ################################################################
 #def moveRestPos(rest_pos, dx, dy, dz):
 #   str_out = ' '
 #   for i in xrange(0,len(rest_pos)) :
 #        str_out= str_out + ' ' + str(rest_pos[i][0]+dx)
 #        str_out= str_out + ' ' + str(rest_pos[i][1]+dy)
 #        str_out= str_out + ' ' + str(rest_pos[i][2]+dz)
 #    return str_out

 #def Error(cur_position, reference):
 #    error = reference - cur_position
 #   return error

 #def Pcontroller(error, gain_P):
 #    effort = gain_P * error
 #    return effort
 #
 #def Icontroller(inte, dt, gain_I):
 #    inte = 0
 #    for i in xrange(1,len(error)):
 #        inte += dt(error[i] - error[i-1])
 #    effort = inte * gain_I
 #    return effort

 #def Dcontroller(error, dt, gain_D):
 #    effort = error/dt * gain_D

 #    return effort
