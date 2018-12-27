#!/usr/bin/env python

from flask import Flask, render_template, flash, request
from wtforms import Form, TextField, TextAreaField, validators, StringField, SubmitField
import rospy
from std_msgs.msg import Float32MultiArray
 
# App config.
DEBUG = True
app = Flask(__name__)
app.config.from_object(__name__)
app.config['SECRET_KEY'] = '7d441f27d441f27567d441f2b6176a'
 
pub = rospy.Publisher('coordinates', Float32MultiArray, queue_size=20)
rospy.init_node('coordinate_publisher', anonymous=False)

class ReusableForm(Form):
    x = TextField('x:', validators=[validators.required()])
    y = TextField('y:', validators=[validators.required()])
 
@app.route("/", methods=['GET', 'POST'])
def hello():
    form = ReusableForm(request.form)
 
    print form.errors
    if request.method == 'POST':
        x=request.form['x']
        y=request.form['y']
        #print x, y
 
    if form.validate():
        flash("coordinates:" + x + "," + y)
        coordinates = [float(x),float(y)]
        pub_coords = Float32MultiArray(data=coordinates)
        rospy.loginfo(pub_coords)
        pub.publish(pub_coords)


    else:
        flash("All the form fields are required. ")
 
    return render_template('hello.html', form=form)
 

if __name__ == "__main__":
    try:
        app.run()
    except rospy.ROSInterruptException:
        pass
