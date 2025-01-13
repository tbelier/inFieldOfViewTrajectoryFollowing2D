from roblib import *
from tools import *
import time
class BlueROV2D():
    def __init__(self,id, x0,y0, theta0,col="blue"):
        self.X = array([[x0],
                        [y0],
                        [theta0]])
        self.tracking = False
        self.kp = 3.
        self.id = id
        self.col = col
        self.FOV = 90
        self.distOfView = 100
        self.LrobInFOV = []
        self.cpt = 0 # compter of iteration without new order
        self.t0 = time.time()
        self.vMax, self.wMax = 30, 1
        self.v,self.w = 0,0
        self.control_input = np.array([[0],
                                       [0],
                                       [0]]) 
        self.turn = np.random.uniform(-pi,pi)
        self.doDisplayQuiver = False
        


    def control(self,u1,u2):
        x,y,theta = self.X.flatten()
        Xp = np.array([[u1*cos(theta)],
                       [u1*sin(theta)],
                       [u2]])
        return Xp
    
    def integration(self,Xp):
        X = self.X + Xp*dt
        return X

    def guidance(self, ax, Lrob): 
        """
        if self.id ==0:
            x,y,theta = self.X.flatten() 
            ox,oy,_ = Lrob[1].X.flatten() 
            err = arctan2(oy-y,ox-x)-theta
            u1, u2 = 20,40*normaliser_angle(arctan2(oy-y,ox-x)-theta)
            print(normaliser_angle(arctan2(oy-y,ox-x)-theta)*180/pi)

        if self.id ==1:
            u1, u2 = 0,0
        """
        u1,u2=10, 0.1*self.turn
        x,y,theta = self.X.flatten()
        if len(self.LrobInFOV) != 0:
            
            idOftrackedRobot = self.LrobInFOV[0] # we take the first in the list
            print(f"{self.col} is Tracking {Lrob[idOftrackedRobot].col}")
            otherX,otherY,otherTheta = Lrob[idOftrackedRobot].X.flatten()
            if self.id < idOftrackedRobot :
                dist, dangle = 5,pi
                targetX,targetY = otherX+dist*cos(otherTheta+dangle), otherY+dist*sin(otherTheta+dangle)

                ax.scatter(targetX,targetY, color=self.col)
                u1, u2 = 5*np.sqrt((targetX-x)**2+(targetY-y)**2),20*normaliser_angle(arctan2(targetY-y,targetX-x)-theta)
                
                self.doDisplayQuiver = True
                print(f"u1 : {u1}, u2 : {u2}")
                       
                       
        
        if self.X.flatten()[0] > Wxmax or self.X.flatten()[0] < Wxmin or self.X.flatten()[1] > Wymax or self.X.flatten()[1] < Wymin:
            u1=10
            u2 = (pi+arctan2(y,x))

        return u1, u2
        
    def checkRobsInFOV(self,Lrob):
        x,y,theta = self.X.flatten()
        dx,dy,dtheta = 0,0,0
        xcamera, ycamera,thetacamera = x+dx, y+dy, theta+dtheta
        FOV = self.FOV*pi/180
        r = self.distOfView

        p1, p2, p3 = [xcamera,ycamera], [xcamera+r*cos(thetacamera+FOV/2),ycamera+r*sin(thetacamera+FOV/2)],[xcamera+r*cos(thetacamera-FOV/2),ycamera+r*sin(thetacamera-FOV/2)]
        plt.scatter(xcamera+r*cos(thetacamera+FOV/2),ycamera+r*sin(thetacamera+FOV/2))
        plt.scatter(xcamera+r*cos(thetacamera-FOV/2),ycamera+r*sin(thetacamera-FOV/2))
        self.LrobInFOV = []
        for k in range(len(Lrob)):
            if k != self.id:
                p = [Lrob[k].X.flatten()[0],Lrob[k].X.flatten()[1]]
                
                if is_same_side(p, p1, p2, p3) and is_same_side(p, p2, p3, p1) and is_same_side(p, p3, p1, p2):
                    self.LrobInFOV.append(k)
                    print(f"{self.id} see : {k}")

    def display(self, ax): 
        #print(self.X)   
        x,y,theta = self.X.flatten()
        draw_rov2D(ax,array([[x],[y],[theta]]), col=self.col, facing = "right")

    def displayFOV(self,ax):
        x,y,theta = self.X.flatten()
        dx,dy,dtheta = 0,0,0
        xcamera, ycamera,thetacamera = x+dx, y+dy, theta+dtheta
        FOV = self.FOV*pi/180
        r = self.distOfView
        triangle_x = [xcamera, xcamera+r*cos(thetacamera+FOV/2),xcamera+r*cos(thetacamera-FOV/2),xcamera]
        triangle_y = [ycamera, ycamera+r*sin(thetacamera+FOV/2),ycamera+r*sin(thetacamera-FOV/2),ycamera]
        ax.fill(triangle_x, triangle_y, self.col, alpha=0.2)

def normaliser_angle(angle):
    # Normalisation dans [-pi, pi]
    return (angle + pi) % (2 * pi) - pi
def fhat(x1,x2,dx,dy):
    return -cos(-arctan2(x2-dy,x1-dx)),sin(-arctan2(x2-dy,x1-dx))

def draw_field(dx,dy):
    Mx = arange(Wxmin, Wxmax,1)
    My = arange(Wymin, Wymax,1)
    X1,X2 = meshgrid(Mx,My)
    VX,VY = fhat(X1,X2,dx,dy)
    quiver(Mx,My,VX,VY)
    return None
    
def simulateAll(ax,Lrob):
    for k in range(len(Lrob)):
        u1,u2 = Lrob[k].guidance(ax,Lrob)
        Xp = Lrob[k].control(u1,u2)
        Lrob[k].X = Lrob[k].integration(Xp)

def checkRobsInFOVAll(Lrob):
    for k in range(len(Lrob)):
        Lrob[k].checkRobsInFOV(Lrob)

def initDisplay():
    ax=init_figure(Wxmin-5,Wxmax+5,Wymin-5,Wymax+5)
    return ax

def displayAll(ax,Lrob):
    clear(ax)
    for k in range(len(Lrob)):
        Lrob[k].display(ax)
        Lrob[k].displayFOV(ax)
        if Lrob[k].doDisplayQuiver:
            dx0,dy0,_ = Lrob[0].X.flatten()
            dx1,dy1,_ = Lrob[1].X.flatten()
            draw_field(dx1,dy1)
    #
    #pause(dt)

def main():
    
    ax = initDisplay()
    colors = ["purple","indigo","blue", "green", "yellow", "orange", "red"]
    Lrob = [BlueROV2D(k,np.random.uniform(Wxmin, Wxmax),np.random.uniform(Wymin, Wymax),np.random.uniform(0, 2*pi),col=colors[k]) for k in range(7)]
    Lrob = [BlueROV2D(0,-10,-5,0,"red"),BlueROV2D(1,0,0,0,"blue")]
    while(True):
        displayAll(ax, Lrob)
        simulateAll(ax,Lrob)
        
        checkRobsInFOVAll(Lrob)

        
    show()

if __name__ == "__main__":
    deltaT = 1
    Wxmin,Wxmax,Wymin,Wymax = -15,15,-15,15
    dt = 0.01
    main()