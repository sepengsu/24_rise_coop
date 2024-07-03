#!/usr/bin/env python

import rospy
import tf
import subprocess

def generate(dot_graph):
    # No need to decode, dot_graph is already a string
    # Generate pdf using graphviz
    command = ['dot', '-Tpdf']
    p = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    pdf_output, err = p.communicate(dot_graph.encode('utf-8'))

    with open('/tmp/frames.pdf', 'wb') as f:
        f.write(pdf_output)

    rospy.loginfo("Wrote frames.pdf to /tmp/frames.pdf")

if __name__ == '__main__':
    rospy.init_node('view_frames')

    listener = tf.TransformListener()

    rospy.sleep(5.0)
    dot_graph = listener.allFramesAsDot()
    
    generate(dot_graph)
