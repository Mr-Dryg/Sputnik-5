import krpc, math, time, json

turn_start_altitude = 250
turn_end_altitude = 150_000
target_altitude = 300_000
g = 9.82

def init():
    vessel = conn.space_center.active_vessel
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), "mean_altitude")
    apoapsis = conn.add_stream(getattr, vessel.orbit, "apoapsis_altitude")
    angle = conn.add_stream(getattr, vessel.auto_pilot, "target_pitch")
    liquid_fuel = conn.add_stream(vessel.resources.amount, 'LiquidFuel')
    return vessel, ut, altitude, apoapsis, angle, liquid_fuel


def new_turn_angle(altitude, turn_start_altitude, turn_end_altitude):
    frac = ((altitude - turn_start_altitude) / (turn_end_altitude - turn_start_altitude))
    return (1 - frac) * 90


def delta_v(Gm, r, a1): # Calculate delta_v (using vis-viva equation)
    a2 = r
    v1 = math.sqrt(Gm*(2/r - 1/a1))
    v2 = math.sqrt(Gm*(2/r - 1/a2))
    return v2 - v1


def burning_time(F, Isp, m0, dv): # Calculate burning time (using rocket equation)
    m1 = m0 / math.exp(dv / (Isp * g))
    fuel_consumption = F / (Isp * g)
    return (m0 - m1) / fuel_consumption


conn = krpc.connect(name="Launch into orbit")
vessel, ut, altitude, apoapsis, angle, liquid_fuel = init()
reference_frame_surface = vessel.orbit.body.reference_frame
vessel.control.throttle = 1.0

time_step_time_to_apo = 2
prev_time_time_to_apo = ut()

# Countdown...
for i in range(3, 0, -1):
    print(f"{i}...")
    time.sleep(1)
start_time = ut()
fligth_time = lambda: ut() - start_time
print("Launch!", fligth_time())

# Save telemetry
time_step = 1
prev_time = fligth_time()
print(prev_time)
data = {}
data["time"] = []
data["altitude"] = []
data["speed"] = []
data["mass"] = []
data["angle"] = []
data["thrust"] = []
data["position"] = []
data["atmosphere_density"] = []


vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)


srbs_separated = 0
turn_angle = 0
state = "Gravity turn"
while True:
    if fligth_time() - prev_time >= time_step:
        prev_time = fligth_time()
        data["time"].append(fligth_time())
        data["altitude"].append(altitude())
        data["speed"].append(vessel.flight(reference_frame_surface).speed)
        data["mass"].append(vessel.mass)
        data["angle"].append(angle())
        data["thrust"].append(vessel.thrust)
        data["position"].append(vessel.position(reference_frame_surface))
        data["atmosphere_density"].append(vessel.flight().atmosphere_density)
#         print(f'''time: {data["time"][-1]}
# altitude: {data["altitude"][-1]}
# speed: {data["speed"][-1]}
# mass: {data["mass"][-1]}
# angle: {data["angle"][-1]}
# thrust: {data["thrust"][-1]}
# position: {data["position"][-1]}
# atmosphere_density: {data["atmosphere_density"][-1]}
# ''')

    # Gravity turn
    if turn_start_altitude <= altitude() <= turn_end_altitude:
        turn_angle = new_turn_angle(altitude(), turn_start_altitude, turn_end_altitude)
        vessel.auto_pilot.target_pitch_and_heading(turn_angle, 90)
    
    if (srbs_separated == 0 and liquid_fuel() < 9500) or (srbs_separated == 1 and liquid_fuel() <= 3240):
        vessel.control.activate_next_stage()
        srbs_separated += 1
        print(f"Separation № {srbs_separated} at {fligth_time()}\n")
        vessel, ut, altitude, apoapsis, angle, liquid_fuel = init()
        vessel.auto_pilot.engage()
        vessel.auto_pilot.target_pitch_and_heading(turn_angle, 90)

    # Disable engines when target apopsis is reached
    elif state == "Gravity turn" and apoapsis() > target_altitude * 0.99:
        vessel.control.throttle = 0.0
        state = "Target apoapsis reached"
        print("Target apoapsis reached", fligth_time())

    elif state == "Target apoapsis reached":
        # Plan circularization burn (using vis-viva equation)
        print("Planning circularization burn", fligth_time())
        Gm = vessel.orbit.body.gravitational_parameter
        r = vessel.orbit.apoapsis
        a1 = vessel.orbit.semi_major_axis
        dv = delta_v(Gm, r, a1)
        node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=dv)

        # Calculate burn time (using rocket equation)
        F = vessel.available_thrust
        Isp = vessel.specific_impulse
        m0 = vessel.mass
        burn_time = burning_time(F, Isp, m0, dv)

        # Orientate ship
        print("Orientating ship for circularization burn", fligth_time())
        vessel.auto_pilot.reference_frame = node.reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)
        vessel.auto_pilot.wait()

        # Wait until burn
        print("Waiting until circularization burn", fligth_time())
        state = "Waiting until circularization burn"
        burn_ut = ut() + vessel.orbit.time_to_apoapsis - burn_time / 2
        lead_time = 15

        conn.space_center.warp_to(max(burn_ut - lead_time, 0))

        # warp_to_time = max(burn_ut - lead_time, 0)

    # elif state == "Waiting until circularization burn" and warp_to_time <= ut():
        # Execute burn
        print("Ready to execute burn", fligth_time())
        state = "Ready to execute burn"
        time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')

    if state == "Ready to execute burn" and ut() - prev_time_time_to_apo >= time_step_time_to_apo:
        prev_time_time_to_apo = ut()
 
    if state == "Ready to execute burn" and time_to_apoapsis() <= burn_time * 0.6:
        print("Executing burn", fligth_time())
        state = "Executing burn"
        vessel.control.throttle = 1.0
        start_burn = ut()
    
    elif state == "Executing burn" and burn_time <= ut() - start_burn:
        print("Fine tuning", fligth_time())
        state = "Fine tuning"
        periapsis = conn.add_stream(getattr, vessel.orbit, "periapsis_altitude")
    
    elif state == "Fine tuning" and periapsis() >= altitude() * 0.98:
        vessel.control.throttle = 0.0
        node.remove()
        break

print("Launch complete", fligth_time())

with open("data.json", 'w') as file:
    json.dump(data, file, indent=4)