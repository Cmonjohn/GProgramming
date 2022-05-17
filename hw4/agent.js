import {scene} from "./main.js"

    constructor(pos, mesh) {
        this.pos = pos.clone();
        this.vel = new THREE.Vector3();
        this.force = new THREE.Vector3();
        this.target = null;
        this.size = 3; // half width
        this.mesh = mesh;
        scene.add(mesh);
        
        this.nbhd = [];
        
        this.MAXSPEED = 50; //最大速率
        this.ARRIVAL_R = 30; //進入arrival模式的半徑
        
        // for orientable agent
        this.angle = 0; //朝向正X軸
    }
    
    update(dt) {
        this.accumulateForce();
        
        // collision
        // for all obstacles in the scene
        let obs = scene.obstacles;
        
        // pick the most threatening one
        let theOne = null;
        let dist = 1e10;
        let vhat = this.vel.clone()
            .normalize();
        const REACH = 50
        const K = 5
        let perp;
        for (let i = 0; i < obs.length; i++) {
            let point = obs[i].center.clone()
                .sub(this.pos) // c-p
            let proj = point.dot(vhat);
            if (proj > 0 && proj < REACH) {
                perp = new THREE.Vector3();
                perp.subVectors(point, vhat.clone()
                    .setLength(proj));
                let overlap = obs[i].size + this.size - perp.length()
                if (overlap > 0 && proj < dist) {
                    theOne = obs[i]
                    dist = proj
                    perp.setLength(K * overlap);
                    perp.negate()
                }
            }
        }
        
        if (theOne)
            this.force.add(perp);
        
        this.vel.add(this.force.clone()
            .multiplyScalar(dt)); //速度更新
        
        
        // ARRIVAL: velocity modulation
        if (this.target !== null) {
            let diff = this.target.clone()
                .sub(this.pos)
            let dst = diff.length();
            if (dst < this.ARRIVAL_R) { //距離越近 速度越慢
                this.vel.setLength(dst);
            }
        }
        
        this.pos.add(this.vel.clone()
            .multiplyScalar(dt))
        this.mesh.position.copy(this.pos)
        
        // for orientable agent
        // non PD version
        if (this.vel.length() > 10) {
            this.angle = Math.atan2(-this.vel.z, this.vel.x)
            this.mesh.rotation.y = this.angle
        }
    }
    
    setTarget(x, y, z) {
        if (this.target)
            this.target.set(x, y, z)
        else
            this.target = new THREE.Vector3(x, y, z);
    }
    
    targetInducedForce(targetPos) { // seek
        return targetPos.clone()
            .sub(this.pos)
            .normalize()
            .multiplyScalar(this.MAXSPEED)
            .sub(this.vel)
    }
    
    accumulateForce() {
        if (this.target !== null)
            this.force.copy(this.targetInducedForce(this.target));
    }
    
    distanceTo(otherAgent) {
        return this.pos.distanceTo(otherAgent.pos)
    }
    
    addNbr(otherAgent) {
        this.nbhd.push(otherAgent)
    }
    
    accumulateForce() {
        
        if (this.target != null)
            this.force.copy(this.targetInducedForce(this.target));
        ////////////////////////////////////////
        // group steer related
        // separation
        if (isSeparation) {
            
            let push = new THREE.Vector3();
            for (let i = 0; i < this.nbhd.length; i++) {
                let point = this.pos.clone()
                    .sub(this.nbhd[i].pos);
                push.add(point.setLength(10 / point.length()));
            }
            this.force.add(push);
            
        }
        
        // coherence
        if (isCohesion) {
            
            if (this.nbhd.length > 0) { // 如果this.nbhd有其他agents
                // 算出 nbhd的平均位置
                // 利用 targetInducedForce （來吸引 this 往平均位置靠近）	
                // 將此力加到 this.force
                let pull = new THREE.Vector3();

                this.nbhd.forEach(function (otherAgent) {
                    pull.add(otherAgent.pos);
                });
                pull.divideScalar(this.nbhd.length);
                pull.y = 0;
                this.force.add(this.targetInducedForce(pull));
            }
            
        }
        
    }