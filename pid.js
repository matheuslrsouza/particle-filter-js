
class PID {

    constructor(path, params) {
        // params = [3.1889941037187843, -1, 0.09000000000123806]
        this.tau_p = params[0]
        this.tau_d = params[1]
        this.tau_i = params[2]
        //0.5584116243577336, 0.7411397001290994, 0.04514526720630067
        // 2.016847699748607, 1.058700280859001, 0.8978036540778822
        // this.tau_p = 2.01
        // this.tau_d = 3.0
        // this.tau_i = 0.89
        this.diff_cte = 0.0
        this.int_cte = 0.0
        this.prev_cte = 0.0
        this.cte = 0.0
        this.index = 0
        this.path = path
    }

    getSteer(position) {

        // compute the CTE
        let x = position[0] 
        let y = position[1]
        
        let normalizer
        
        let rx
        let ry
        
        let delta_x
        let delta_y

        let on_path = false

        while (!on_path) {
        
            let xy1 = this.fromIndexToPixel(this.index, this.path)
            let px1 = xy1[0]
            let py1 = xy1[1]
            
            let xy2 = this.fromIndexToPixel(this.index + 1, this.path)
            let px2 = xy2[0]
            let py2 = xy2[1]
            
            delta_x = px2 - px1
            delta_y = py2 - py1
            
            rx = x - px1
            ry = y - py1
            
            normalizer = delta_x * delta_x + delta_y * delta_y
            // console.log('normalizer', normalizer)
            // console.log('u', (rx * delta_x + ry * delta_y) / normalizer)
            if (normalizer != 0) {
                let u = (rx * delta_x + ry * delta_y) / normalizer
                if (u > 1) {
                    if (this.index == this.path.length - 2) {
                        break
                    }
                    this.index += 1
                } else {
                    on_path = true
                }
            } else {
                on_path = true
            }
        }
        // console.log(this.index)
        
        if (this.index >= this.path.length - 1) {
            console.log('ultrapassou!')
            return 
        }
        
        if (normalizer != 0) {
            this.cte = (ry * delta_x - rx * delta_y) / Math.sqrt(normalizer)
        }
                
        this.diff_cte = this.cte - this.prev_cte
        this.int_cte += this.cte

        this.prev_cte = this.cte
        
        let steer = -this.tau_p * this.cte -this.tau_d * this.diff_cte * -this.tau_i * this.int_cte
        
        return steer
    
    }
    
    fromIndexToPixel(i, matrix) {
        let x = matrix[i][0] * width / n_rows
        let y = matrix[i][1] * height / n_cols
        return [x, y]
    }

}