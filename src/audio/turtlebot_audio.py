#! /usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class TurtlebotAudio( object ):

   def __init__( self ):
      #self.soundhandle = SoundClient( blocking = True)
      # ros hydro doesn't support blocking option
      self.soundhandle = SoundClient()

   def say( self, s, voice = 'voice_kal_diphone', volume = 1.0 ):
      rospy.loginfo( 'Saying: \'%s\'', s )
      rospy.loginfo( 'Voice: %s', voice )
      rospy.loginfo( 'Volume: %s', volume )
      self.soundhandle.say( s, voice )

if __name__ == '__main__':

   rospy.init_node( 'sound_test' )
   speaker = TurtlebotAudio()

   speaker.say( 'Hello world' )
   speaker.say( 'Bye world' )

   rospy.spin()

