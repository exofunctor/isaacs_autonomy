#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify
import pandas as pd
import numpy as np

app = Flask(__name__)

@app.route('/')
def home():
	return render_template("home.html")

@app.route("/about")
def about():
	return render_template("about.html")

@app.route("/demo", methods=['GET', 'POST'])
def demo():
	
	if request.method == 'POST':

		valid_input = True
		print("Recieved order")

		name = request.form['name']
		print("Name", name)
		email = request.form['email']
		print("Email", email)
		address = request.form['address']
		print("Addre", address)
		lat = request.form['lat']
		print("Lat", lat)
		lng = request.form['lng']
		print("Lng", lng)
		item = request.form['item']
		print("Item", item)


	return render_template("demo.html")

if __name__ == "__main__":
	app.run(debug=True)



