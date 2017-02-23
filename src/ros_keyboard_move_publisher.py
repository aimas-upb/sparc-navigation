#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

#keystrokes
import curses

def move_publisher():
	pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
	rospy.init_node('test_move_node', anonymous=True)
	
	cmd = PoseStamped()
	cmd.header.frame_id = "base_footprint"

	step = 0.1
	
	stdscr = curses.initscr()
	curses.cbreak()
	stdscr.keypad(1)

	stdscr.addstr(0, 10, "Hit 'q' to quit")
	stdscr.refresh()

	key = ''
	while key != ord('q'):
		key = stdscr.getch()

		stdscr.refresh()

		if key == curses.KEY_UP: 
			stdscr.addstr(2, 20, "Forward")
			cmd.pose.position.x = step
			pub.publish(cmd)

		elif key == curses.KEY_DOWN: 
			stdscr.addstr(3, 20, "Backward")
			cmd.pose.position.x = -step
			pub.publish(cmd)

		elif key == curses.KEY_LEFT:
			stdscr.addstr(4, 20, "Left")
			cmd.pose.position.y = step
			pub.publish(cmd)

		elif key == curses.KEY_RIGHT:
			stdscr.addstr(5, 20, "Right")
			cmd.pose.position.y = -step
			pub.publish(cmd)

		elif key == ord('k'):
			step += 0.1
		elif key == ord('j'):
			step -= 0.1

		#reset
		cmd.pose.position.x = 0.0
		cmd.pose.position.y = 0.0

	curses.endwin()
		
if __name__ == '__main__':
	try:
		move_publisher()
	except rospy.ROSInterruptException:
		pass
