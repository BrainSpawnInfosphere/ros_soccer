################################
# Battery Message
# The robot tracks battery useage (low level) as far
# as power used and level remaining
#
Header header
float32 volts		# current volts (V) 
float32 amps		# current amps (mA)
float32 power  # total power used (Whr)
float32 level       # battery level remaining (%)
