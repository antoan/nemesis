
cdef public class ThunderBorg [object c_ThunderBorg, type c_Thunderborg_t]:

    
    cdef public double SetMotor1(self, double power):
        return power

    cdef public SetMotor2(self, double power):
        pass
    
    cdef public Init(self):
        pass

cdef public ThunderBorg buildThunderBorg():
    return ThunderBorg()

cdef public SetMotor1Wrapper(ThunderBorg borg, double power):
    borg.SetMotor1(power)
