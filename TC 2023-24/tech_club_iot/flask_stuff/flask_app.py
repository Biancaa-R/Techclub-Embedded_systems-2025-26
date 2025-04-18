from flask import Flask,redirect,url_for,render_template
app=Flask(__name__)

'''@app.route("/home/<name>")

def home(name):
    return f"<h1>Hi {name}</h1>"'''

@app.route("/home")

def home():
    return render_template("home.html")


@app.route("/about")
def about():
    #Does not depend on the name
    return "<h1>Hi this is a website with flask about us page</h1>"

if __name__=="__main__":
    app.run(debug=True)