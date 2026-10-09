-- L0 Lidar
-- By:      Lluis Ribas-Xirgo
--          Universitat Autonoma de Barcelona
-- Date:    2023-2025
-- License: Creative Commons, BY-SA = attribution, share-alike
--          [https://creativecommons.org/licenses/by-sa/4.0/]

servo_sensor = "servo_sensor"
sensor_ping  = "lidar_ping"
sensor_echo  = "lidar_echo"

Lidar = {
    A = nil,     -- Input target angle, must be kept constant
    r = nil,     -- Input request obstacle distance
    E = nil,     -- Input obstacle distance
    G = nil,     -- Servo position, only for characterization
    T = nil,     -- Time [s]
    dT = nil,    -- Time step [s]
    W = nil,     -- Sensor servo speed in degrees per cyle
    state = {},
    B = {},      -- Begin time
    D = {},      -- Delay
    Y = {},      -- Yaw
    Z = {},      -- Control variable for yaw Y 
    Q = {},      -- Output obstacle distance
    p = {},      -- Pending request
    u = {},      -- Distance update 
    s = false,   -- Send ping
    characterize = nil -- Set to true if starting with characterization
}

function Lidar.Y2Z(A)
  if A==360 then A=0 else A=math.floor(1500-A*160/90) end
  return A
end -- function
function Lidar:update()
  self.state.curr = self.state.next
  self.B.curr = self.B.next
  self.D.curr = self.D.next
  self.Y.curr = self.Y.next
  self.Z.curr = self.Z.next
  self.Q.curr = self.Q.next
  self.p.curr = self.p.next
  self.u.curr = self.u.next
  self.s = (self.state.curr=="PING")
end -- function
function Lidar:init()
  self.W = 60; -- deg/s (aprox. half of reality)
  self.T = sim.getSimulationTime()
  self.state.next = "ROTATE"
  self.B.next = self.T
  self.D.next = 90/self.W
  self.Y.next = 0
  self.Z.next = self.Y2Z(0)
  self.Q.next = 0
  self.p.next = false
  self.u.next = false
  self.s = false
  if self.characterize then
    self.state.next = "cROTATE"
    self.D.next = 0 -- [s]
    self.G = 0
    self.lidar = sim.getObject("../sensor_motor/rotor_base/sensor_hold")
    if self.lidar then
      local eulerAngles = sim.getObjectOrientation(self.lidar, sim.handle_world)
      if eulerAngles then -- [-90, -180/180, +90] --> [-90, 0, +90]
        self.G = math.floor(eulerAngles[3]*180/math.pi)
        if self.G<0 then self.G = -self.G-180 else self.G = -self.G+180 end
      end -- if
    end -- if
    self.Y.next = self.G+90
    self.Z.next = self.Y2Z(self.G+90)
  end -- if
  self:update()
end -- function
function Lidar:monitor()
  print(string.format("Lidar: state=%s p=", self.state.curr or "<unset>")) 
  if self.p.curr then print("true") else print("false") end
  print(string.format(" B= %.4fs D= %.4fs Y= %ideg Z=%ius Q=%icm u=",
      self.B.curr, self.D.curr, self.Y.curr, self.Z.curr, self.Q.curr)) 
  if self.characterize then 
    if self.u.curr then print("true") else print("false") end
    println(string.format(" W= %.2fdeg/s", self.W))
  else
    if self.u.curr then println("true") else println("false") end
  end -- if
end -- function
function Lidar:cmonitor()
    if self.state.prev~=self.state.curr then
        self.state.prev = self.state.curr; self.monitor(self)
    end -- if
end -- function
function Lidar:read_inputs(A, r)
  if A~=nil then
      if A<-90 then self.A = -90
      elseif A>90 then self.A = 90
      else self.A = A
      end -- if chain
  else
      self.A = 0
  end -- if
  if r~=nil then self.r = r else self.r = false end
  local echo = sim.getInt32Signal(sensor_echo)
  if echo==nil or echo<0 then
      self.E = nil
  else
      self.E = echo -- [cm]
  end -- if
  self.T = sim.getSimulationTime()
  self.dT = sim.getSimulationTimeStep()
  --print(string.format("@%.4fs: L0Lidar.E = ", self.T))
  --if self.E then println(string.format("%d", echo)) else println("nil") end
  if self.characterize then
    self.G = 0
    if self.lidar then
      local eulerAngles = sim.getObjectOrientation(self.lidar, sim.handle_world)
      if eulerAngles then
        self.G = math.floor(eulerAngles[3]*180/math.pi)
        if self.G<0 then self.G = -self.G-180 else self.G = -self.G+180 end; println("Lidar.G ="..self.G)
      end -- if
    end -- if
  end -- if
end -- function
function Lidar:plan()
  if self.state.curr=="ROTATE" then
    self.p.next = self.p.curr or self.r -- in case some request interrupts a previous one
    if self.T-self.B.curr>=self.D.curr then
      self.Z.next = self.Y2Z(360)
      self.state.next = "PING"
    end -- if
  elseif self.state.curr=="PING" then
    self.p.next = self.p.curr or self.r
    self.u.next = false -- a single positive pulse after request
    if self.A~=self.Y.curr then
      self.B.next = self.T
      self.D.next = math.abs(self.A-self.Y.curr)/self.W
      self.Y.next = self.A
      self.Z.next = self.Y2Z(self.A)
      self.state.next = "ROTATE"
    else
      self.state.next = "ECHO"
    end -- if chain                                
  elseif self.state.curr=="ECHO" then
    if self.r then
      self.p.next = true
      self.state.next = "PING"         
    elseif self.E~=nil then
      self.p.next = false
      self.u.next = self.p.curr
      self.Q.next = self.E
      self.state.next = "PING"
    end -- if
  elseif self.state.curr=="cROTATE" then
    self.p.next = self.p.curr or self.r -- in case some request interrupts a previous one
    if math.abs(self.G-self.Y.curr)<1.0 then
      self.Z.next = self.Y2Z(360)
      self.W = 90.0/(self.T-self.B.curr)
      self.W = self.W/2
      self.state.next = "ROTATE"
      self.B.next = self.T
      self.D.next = 90/self.W
      self.Y.next = 0
      self.Z.next = self.Y2Z(0)
      self.Q.next = 0
      self.p.next = false
      self.u.next = false
      self.s = false
    end -- if
  else -- Stop state or error
    self.state.next = "STOP"
  end -- if..elseif
end -- function
function Lidar:write_outputs()
  if self.s then
    sim.setInt32Signal(sensor_ping, 1)
  else
    sim.setInt32Signal(sensor_ping, 0)
  end
  if self.Z.curr~=360 then sim.setInt32Signal(servo_sensor, self.Z.curr) end
end -- function
function Lidar:get_distance()
    return self.Q.curr
end -- function
function Lidar:get_update()
    return self.u.curr
end -- function
function Lidar:active()
  return self.state.curr=="ROTATE" or
         self.state.curr=="cROTATE" or
         self.state.curr=="PING" or
         self.state.curr=="ECHO"
end -- function
