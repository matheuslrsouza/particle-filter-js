
class Particle {

    constructor(x, y, angle) {
        this.measurements = []
        this.pos = createVector(x, y)
        this.dir = p5.Vector.fromAngle(angle)
        this.normalizeAngle()
        this.dir.normalize()
    }

    // normalize the angle in order to get only positive values
    normalizeAngle() {
        let angle = this.dir.heading()
        while (angle < 0) {
            angle += 2 * PI
        }
        this.dir = p5.Vector.fromAngle(angle)
    }

    show() {
        if (window['DEBUG']) {
            push()
            translate(this.pos.x, this.pos.y)
            fill(0, 255, 0, 20)
    
            let triangleSize = 15
            rotate(this.dir.heading())
            triangle(0, triangleSize / 4, 0, -triangleSize / 4, triangleSize, 0)
            
            pop()
        }
    }

    predict(delta_t, std_pos, vel, yaw_rate) {
        let theta = this.dir.heading()  
        if (yaw_rate > 0.0001 || yaw_rate < -0.0001) {
            this.pos.x = this.pos.x + (vel / yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta));
            this.pos.y = this.pos.y + (vel / yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t));
            this.dir = p5.Vector.fromAngle(theta + yaw_rate * delta_t);
        } else {
            this.pos.x = this.pos.x + vel * delta_t * cos(theta)
            this.pos.y = this.pos.y + vel * delta_t * sin(theta)
        }

        this.pos.x = randomGaussian(this.pos.x, std_pos[0])
        this.pos.y = randomGaussian(this.pos.y, std_pos[1])
        this.dir = p5.Vector.fromAngle(randomGaussian(this.dir.heading(), std_pos[3]))

        this.normalizeAngle()
    }

    updateWeights(robotMeasurements, std_landmark) {

        let final_weight = 1.0;        

        // console.log('--- ini --- ')

        // para cada observação do robo, verificamos:
        //for (let rMeasurement of robotMeasurements) {
        for (let measurement of this.measurements) {
            let rMeasurement = measurement.rMeasurement
            let pMeasurement = measurement.pMeasurement
            
            // 1) qual ponto medido pela particula está mais perto do observado
            // nearest neighbor

            // with polar system, calculate an imaginary x and y measurement for robot
            let robotX = cos(rMeasurement.theta + rMeasurement.heading) * rMeasurement.r
            let robotY = sin(rMeasurement.theta + rMeasurement.heading) * rMeasurement.r

            // do the same for the particle measurement in angle of the particle heading
            let particleX = cos(pMeasurement.theta) * pMeasurement.r
            let particleY = sin(pMeasurement.theta) * pMeasurement.r

            // console.log("(" + robotX, robotY + ")", "(" + particleX, particleY + ")")

            // 2) calcula o peso dado uma distribuição gaussiana multi variavel
            // quanto mais distante o ponto estiver do medido pelo robo, maior a 
            // propabilidade de ser excluída

            // applying Multivariate_normal_distribution
            // calculate normalization term
            const gauss_norm = (1.0/(2.0 * PI * std_landmark[0] * std_landmark[1]));

            // calculate exponent
            const mu_x = particleX;
            const mu_y = particleY;
            const exponent = (Math.pow(robotX - mu_x, 2))/(2 * Math.pow(std_landmark[0], 2)) + 
                                            (Math.pow(robotY - mu_y, 2))/(2 * Math.pow(std_landmark[1], 2));

            // calculate weight using normalization terms and exponent
            final_weight *= gauss_norm * Math.exp(-exponent);
            
        }

        // console.log(final_weight)
        // console.log('--- fim --- ')

        return final_weight
    }

    check(walls, robotMeasurements) {
        // console.log('--- ini check --- ')
        for (let m of robotMeasurements) {
            let ray = new Ray(createVector(this.pos.x, 
                                            this.pos.y),
                                            p5.Vector.fromAngle(this.dir.heading() + m.theta))

            // console.log(ray.dir.heading())
            

            // using the created ray, checks if there is wall
            let closest
            let minDist = Infinity
            for (wall of walls) {
                let point = ray.intersects(wall)
                if (point) {
                    // get the distance of particle and the wall for the current ray
                    const d = dist(this.pos.x, this.pos.y, point.x, point.y)
                    if (d < minDist) {
                        closest = point
                        minDist = d
                    }
                }
            }
            // for each ray only gets the closest wall (avoid overshooting)
            if (closest) {
                // stroke(255, 100)
                // line(this.pos.x, this.pos.y, closest.x, closest.y)
                this.measurements.push({
                    rMeasurement: m, 
                    pMeasurement: {
                        r: minDist, theta: ray.dir.heading()
                    }
                })
            }

        }
        // console.log('--- fim check --- ')
    }

}