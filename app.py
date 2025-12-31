from flask import Flask, render_template, jsonify, request

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/action/hello", methods=["POST"])
def hello_action():
    return jsonify({"message": "Hello world!"})

if __name__ == "__main__":
    app.run(debug=True)
