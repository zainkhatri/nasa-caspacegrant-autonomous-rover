import React, { useRef, useEffect } from 'react';
import p5 from 'p5';

const RoverSimulation = () => {
  const sketchRef = useRef();

  useEffect(() => {
    const sketch = (p) => {
      let rover;
      let waypoints = [
        {x: 100, y: 100},
        {x: 300, y: 200},
        {x: 500, y: 400}
      ];
      let currentWaypointIndex = 0;
      let sensorThreshold = 50;

      p.setup = () => {
        p.createCanvas(800, 600).parent(sketchRef.current);
        rover = new Rover(p.width / 2, p.height - 50); // Initialize rover
      };

      p.draw = () => {
        p.background(220);

        // Draw waypoints
        for (let wp of waypoints) {
          p.fill(0, 255, 0);
          p.ellipse(wp.x, wp.y, 10, 10);
        }

        rover.show();
        rover.moveTowards(waypoints[currentWaypointIndex]);

        if (rover.reachedWaypoint(waypoints[currentWaypointIndex])) {
          currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.length; // Move to next waypoint
        }

        // Obstacle detection simulation
        if (rover.x < sensorThreshold || rover.x > p.width - sensorThreshold || 
            rover.y < sensorThreshold || rover.y > p.height - sensorThreshold) {
          rover.stop();
        }
      };

      class Rover {
        constructor(x, y) {
          this.x = x;
          this.y = y;
          this.speed = 2;
          this.angle = 0;
          this.size = 20;
        }

        show() {
          p.fill(150, 0, 255);
          p.push();
          p.translate(this.x, this.y);
          p.rotate(this.angle);
          p.rectMode(p.CENTER);
          p.rect(0, 0, this.size * 2, this.size);
          p.pop();

          // Show sensor ranges (left, center, right)
          this.showSensors();
        }

        showSensors() {
          p.stroke(255, 0, 0);
          p.line(this.x, this.y, this.x + p.cos(this.angle) * sensorThreshold, this.y + p.sin(this.angle) * sensorThreshold); // Center sensor
          p.line(this.x, this.y, this.x + p.cos(this.angle + p.QUARTER_PI) * sensorThreshold, this.y + p.sin(this.angle + p.QUARTER_PI) * sensorThreshold); // Right sensor
          p.line(this.x, this.y, this.x + p.cos(this.angle - p.QUARTER_PI) * sensorThreshold, this.y + p.sin(this.angle - p.QUARTER_PI) * sensorThreshold); // Left sensor
        }

        moveTowards(target) {
          let targetAngle = p.atan2(target.y - this.y, target.x - this.x);
          this.angle = targetAngle; // Adjust rover's angle towards waypoint

          // Move forward
          this.x += p.cos(this.angle) * this.speed;
          this.y += p.sin(this.angle) * this.speed;
        }

        stop() {
          this.speed = 0; // Stop the rover
        }

        reachedWaypoint(target) {
          let distance = p.dist(this.x, this.y, target.x, target.y);
          return distance < 5; // Threshold for considering the waypoint reached
        }
      }
    };

    const myP5 = new p5(sketch);

    return () => {
      myP5.remove(); // Clean up the p5 instance on component unmount
    };
  }, []);

  return (
    <div>
      <h1>Rover Simulation</h1>
      <div ref={sketchRef}></div>
    </div>
  );
};

export default RoverSimulation;
