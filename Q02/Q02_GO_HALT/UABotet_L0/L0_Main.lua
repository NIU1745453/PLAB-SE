-- L0 Open-loop control of UABotet
-- By:      Lluis Ribas-Xirgo
--          Universitat Autonoma de Barcelona
-- Date:    September 2023-2025
-- License: Creative Commons, BY-SA = attribution, share-alike
--          [https://creativecommons.org/licenses/by-sa/4.0/]

-- L0 Main
-- By:      Lluis Ribas-Xirgo
--          Universitat Autonoma de Barcelona
-- Date:    September 2023
-- License: Creative Commons, BY-SA = attribution, share-alike
--          [https://creativecommons.org/licenses/by-sa/4.0/]

-- L0 UI interface
L0_UI = {}
L0_UI.xml = [[
  <ui title="L0 User Interface" resizable="true"
      style="background-color:lightBlue"
  >
    <group layout="hbox" flat="true">
    <group layout="vbox" flat="true">
    <group layout="hbox" flat="true">
    <label text="Angle" />
    <edit	id="110" value="0"
      on-editing-finished="L0_UI.updateAtext"
      style="background-color:white"
    />
    <label text="deg" />
    </group>
    <hslider id="115" minimum="-90" maximum="90" value="0" 
      tick-interval="10" tick-position="below" on-change="L0_UI.updateAslider" />
    <group layout="hbox" flat="true">
    <label text="Distance" />
    <edit	id="120" value="0"
      on-editing-finished="L0_UI.updateDtext"
      style="background-color:white"
    />
    <label text="cm" />
    </group>        
    <hslider id="125" minimum="0" maximum="255" value="0" 
      tick-interval="10" tick-position="below" on-change="L0_UI.updateDslider" />
    <group layout="hbox" flat="true">
    <button id="1" text="GO" style="background-color:lightGreen"
      on-click="L0_UI.GO" />
    <button id="4" text="HALT" style="background-color:red"
      on-click="L0_UI.HALT" />
    </group>
    <group layout="hbox" flat="true">
    <button id="2" text="LIDAR" style="background-color:lightGreen"
      on-click="L0_UI.LIDAR" />
    <button id="3" text="RESUME" style="background-color:lightGreen"
      on-click="L0_UI.RESUME" />
    </group>
    </group>

    <group layout="vbox" flat="true">
    <label text="From L0: "/>
    <text-browser id="40" text="..."
       type="plain" read-only="true" style="align:left"
    />
    <!--tree id="40" show-header="false" style="align:left">
      <row> <item> ... </item> </row>
    </tree-->
    </group>

    </group>
  </ui>
]]
if sim then
  L0_UI.handle = simUI.create(L0_UI.xml)
  simUI.setPosition(L0_UI.handle, 50, 450)
end -- if
L0_UI.angle = 0
L0_UI.radius = 0
L0_UI.C = nil
L0_UI.L0msg = "..."
L0_UI.updateAtext = function(uiHandle, id, newValue)
  local angle = tonumber(newValue)
  if angle then
    if angle < -90 then angle = -90 end
    if angle > 90 then angle = 90 end
    L0_UI.angle = angle
    simUI.setSliderValue(uiHandle, 115, angle)
  else
    angle = L0_UI.angle
  end
  simUI.setEditValue(uiHandle, 110, string.format("%d", angle))
end
L0_UI.updateAslider = function(uiHandle, id, newValue)
  L0_UI.angle = newValue
  simUI.setEditValue(uiHandle, 110, string.format("%d", newValue))
end
L0_UI.updateDtext = function(uiHandle, id, newValue)
  local radius = tonumber(newValue)
  if radius then
    if radius < 0 then radius = 0 end
    if radius > 255 then radius = 255 end
    L0_UI.radius = radius
    simUI.setSliderValue(uiHandle, 125, radius)
  else
    radius = L0_UI.radius
  end
  simUI.setEditValue(uiHandle, 120, string.format("%d", radius))
end
L0_UI.updateDslider = function(uiHandle, id, newValue)
  L0_UI.radius = newValue
  simUI.setEditValue(uiHandle, 120, string.format("%d", newValue))
end
L0_UI.GO = function(uiHandle, id)
  L0_UI.C = string.format("1 %d %d", L0_UI.angle, L0_UI.radius)
end
L0_UI.HALT = function(uiHandle, id) L0_UI.C = "4" end
L0_UI.LIDAR = function(uiHandle, id) L0_UI.C = "2" end
L0_UI.RESUME = function(uiHandle, id) L0_UI.C = "3" end
L0_UI.setA = function(self, angle)
  if angle then
    if angle < -90 then angle = -90 end
    if angle > 90 then angle = 90 end
    self.angle = angle
    simUI.setLabelText(self.handle, 110, string.format("%d", angle))
    simUI.setSliderValue(self.handle, 115, angle)
  end
end
L0_UI.setD = function(self, radius)
  if angle then
    if radius < 0 then radius = 0 end
    if radius > 255 then radius = 255 end
    self.radius = radius
    simUI.setLabelText(self.handle, 120, string.format("%d", radius))
    simUI.setSliderValue(self.handle, 125, radius)
  end
end
L0_UI.getC = function(self)
  local C = self.C
  self.C = nil
  return C
end
L0_UI.appendM = function(self, M)
  self.L0msg = M..'\n'..self.L0msg
  simUI.setText(self.handle, 40, self.L0msg)
end


-- Aqui se ha de hacer la definicion de la funcion sgn(x)
function sgn(x)
  if x<0 then
    return -1
  elseif x==0 then
    return 0
  elseif x>0 then
    return 1
  end
end --function

-- L0 CONTROLLER
L0Main = {
  V,          -- constant, linear velocity
  W,          -- constant, angular velocity
  I,          -- input, instruction
  T,          -- input, current time
  state = {},
  B = {},     -- Begin time 
  A = {},     -- Absolute angle rotation
  S = {},     -- Space/distance to move ahead
  L = {},     -- DC value for left motor
  R = {},     -- DC value for right motor
  M = {}      -- Message to user/upper-level controllers
}
  
function L0Main:init()
  self.V =  2.925 -- cm/s  
  self.W = 31.85  -- deg/s 
  self.I = {0, 0, 0} -- Instruction table
  self.T = 0         -- Time
  self.state = {}; self.state.curr = "LISTEN"; self.state.next = "LISTEN"; self.state.prev="NONE"
  self.B.curr = 0; self.B.next = 0 
  self.A.curr = 0; self.A.next = 0
  self.S.curr = 0; self.S.next = 0 
  self.L.curr = 0; self.L.next = 0 
  self.R.curr = 0; self.R.next = 0
  self.M.curr = nil; self.M.next = nil
end -- function

function L0Main:monitor()
  print(string.format("L0Main: state=%s M=", self.state.curr))
  if self.M.curr then println(self.M.curr) else println("nil") end
  println(string.format("        A=%3i S=%3i L=%4i R=%4i",
          math.floor(self.A.curr), math.floor(self.S.curr), self.L.curr, self.R.curr))
end -- function

function L0Main:cmonitor()
  if self.state.prev~=self.state.curr then
    self.state.prev = self.state.curr; self:monitor()
  end -- if
end -- function

function L0Main:update()
  -- A completar
  self.state.curr = self.state.next
  self.B.curr = self.B.next
  self.A.curr = self.A.next
  self.S.curr = self.S.next
  self.L.curr = self.L.next
  self.R.curr = self.R.next
  self.M.curr = self.M.next 
  
end -- function

function L0Main:read_inputs()
  local C = L0_UI:getC()
  if sim then
    self.T = sim.getSimulationTime()
  else -- console input
    self.T = self.T + 0.2
    io.write(string.format("> T = %.2fs or ...", self.T))
    local newT = tonumber(io.read())
    if newT and newT>self.T then self.T = newT end
    io.write(string.format("> I = "))
    C = io.read()
  end -- if
  if C and #C>0 then
      local words = {}
      for w in string.gmatch(C, "-?%d+") do table.insert(words, w) end
      self.I[1] = tonumber(words[1])
      if self.I[1] then
        if self.I[1]==1 then -- GO
          if #words>1 then self.I[2] = tonumber(words[2]) end
          if #words>2 then self.I[3] = tonumber(words[3]) end
        else
          self.I[2] = 0; self.I[3] = 0
        end -- if
      end -- if
  else
      self.I[1] = 0 -- no command received
  end -- if
end -- function

function L0Main:write_outputs()
  if sim then
    if self.M.curr then L0_UI:appendM(self.M.curr) end
    if self.L and self.L.curr then
      sim.setInt32Signal("DC_left", self.L.curr)
      sim.setInt32Signal("DC_right", self.R.curr)
    end -- if
  else
    io.write(string.format("< State= %s, L= %i, R= %i, M= ", self.state.curr, self.L.curr, self.R.curr))
    if self.M.curr then io.write(self.M.curr.."\n") else io.write("nil\n")  end
  end -- if 
end -- function

function L0Main:plan()
  if not sim and self.I[1]==nil then self.state.curr = "STOP" end
  if self.state.curr=="LISTEN" then
  -- A completar
    if self.I[1]==0 then
      self.M.next = nil
    elseif self.I[1]==4 then
      self.M.next = "E HALT LISTEN?"
    elseif self.I[1]==1 then
      self.state.next ="TURN"
      self.B.next = self.T
      self.A.next = math.abs(self.I[2])
      self.S.next = self.I[3]
      self.L.next = sgn(self.I[2])*100
      self.R.next = -sgn(self.I[2])*100
      self.M.next = nil
    elseif self.I[1]>1 then
      self.M.next = "E Unknown"
    end -- if..elseif..elseif
  elseif self.state.curr=="TURN" then
    if self.I[1]==4 then
      local val = self.A.curr - (self.T-self.B.curr)*self.W
      if val < 0 then val = 0 end
      self.M.next = string.format("D TURN HALTed, %d deg to go",math.floor(val))
      self.state.next = "LISTEN"
      self.L.next = 0
      self.R.next = 0
    elseif not ((self.T-self.B.curr)*self.W < self.A.curr) then
      self.state.next = "FWD"
      self.B.next = self.T
      self.L.next = 100
      self.R.next = 100
    end
  elseif self.state.curr=="FWD" then
    if self.I[1]==4 then
      local val = self.S.curr - (self.T-self.B.curr)*self.V
      if val < 0 then val = 0 end
      self.M.next = string.format("D TURN HALTed, %d cm to go",math.floor(val))
      self.state.next = "LISTEN"
      self.L.next = 0
      self.R.next = 0
    elseif not((self.T-self.B.curr)*self.V<self.S.curr) then
      self.state.next = "LISTEN"
      self.L.next = 0
      self.R.next = 0
      self.M.next = "D OK GO"
    end --if
  else -- Error
    self.state.next = "STOP"
  end -- if..ifelse
end -- function


function L0Main:active()
  return self.state.curr=="LISTEN" or self.state.curr=="TURN" 
      or self.state.curr=="FWD"
end -- function

if not sim then -- LOCAL SIMULATION ENGINE
  L0Main:init()
  C = 0 -- Cycle counter
  while L0Main:active() do
    io.write(string.format("Cycle %d:\n", C)) 
    --L0Main:monitor()
	L0Main:write_outputs()
    L0Main:read_inputs()
    L0Main:plan()
    C = C + 1 -- next cycle
    L0Main:update()
    io.write("----------------------------------------\n")
  end -- while
  io.write(string.format("Cycle %d:\n", C)) 
  L0Main:write_outputs() -- outputs at error or STOP state
  io.write("Program exited!\n")
end -- if