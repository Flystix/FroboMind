# Telemetry message to be transfered from Airframe to Groundstation
Header header		
fmVector3 orientation	# Orientation vector (ϕ, θ, ψ)
fmVector3 position	# Position vectos (Pn, Pe, Alt)
float64 airspeed	# Speed through air (Pitot measure) [m/s]
float64 truespeed	# True airspeed (wind compendsated) [m/s]
float64 groundspeed	# Speed over ground [m/s]
float64 battery		# Battery voltage [mV] - ALT: State of Charge [%]
float64 cpuload		# Load on CPU [%]
float64 memutil		# Memory utilization [%]
uint8 gps_fix		# GPS fix: 0=Invalid, 1=GPS, 2=DGPS,4=RTK fixed, 5=RTK float 
uint8 gps_sats		# Number of satellites in view
