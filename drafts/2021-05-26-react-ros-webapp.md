---
title: Developing React ROS webapp for robot 

date: 2021-03-14 
categories: [ros, react webapp]
tags: [react, ros, webapp]
math: true
mermaid: true

---


## Setup for React app for ROS developers

Create React app in visual studio code as shown in this [link](https://reactjs.org/docs/create-a-new-react-app.html).

Note: First install `npm` in ubuntu to use `npx` command to create-react-app.

> sudo apt install npm 
> npx npx create-react-app react-ros-webapp
> cd my-app
> npm start

`react-ros-webapp` is the name of our app. 

`npm start` gives the following output 

```sh

Compiled successfully!

You can now view react-ros-webapp in the browser.

  Local:            http://localhost:3000
  On Your Network:  http://10.0.2.15:3000

Note that the development build is not optimized.
To create a production build, use npm run build.

```

Follow the link to see changes to your react web app. 

# Setting up the bootstrap theme 

To setup the bootstrap theme go to this [link](https://bootswatch.com/).
Download a bootstrap theme of your choice and put it in the `bootstrap folder` in the `src folder` of the react app folder. 

And add th following import statement to include the bootstrap theme of your choice in `index.js` inside **src folder**.

```javascript
import './bootstrap/lux-bootstrap.min.css';
```
Note: do `npm start` in the visual studio code terminal  to see the changes in the  webapp.
# 



