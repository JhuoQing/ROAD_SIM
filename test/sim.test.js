const RS = require('../src/sim.js');
function cross(lanes, signal, limit=50){
  const net = RS.createNet();
  RS.addSegment(net,-250,0,250,0,{lanes,limit,oneway:false});
  RS.addSegment(net,0,-250,0,250,{lanes,limit,oneway:false});
  for (const n of net.nodes){ if (RS.nodeDegree(net,n.id)===4 && signal) n.sig = RS.defaultSig(); }
  return net;
}
function run(name, net, opt, secs){
  const sim = new RS.Sim(net, opt);
  let minGap = 1e9, maxV=0, t0=Date.now(), maxN=0, overlaps=0;
  const steps = secs*10;
  for (let i=0;i<steps;i++){
    sim.step();
    if (i%10===0){
      maxN=Math.max(maxN, sim.vehicles.length);
      for (const e of sim.g.edges){
        const all=[]; for (const L of e.lanesArr) for (const x of L) all.push(x);
        for (const x of all){ if (x.v<-1e-9||isNaN(x.v)||isNaN(x.s)||isNaN(x.lat)) throw new Error('bad state'); maxV=Math.max(maxV,x.v); }
        for (let a=0;a<all.length;a++) for (let b2=a+1;b2<all.length;b2++){
          const p1=all[a], p2=all[b2];
          const dl=Math.abs(p1.lat-p2.lat)-(p1.wid+p2.wid)/2;
          const ds=Math.abs(p1.s-p1.len/2-(p2.s-p2.len/2))-(p1.len+p2.len)/2;
          if (p1.lane===p2.lane){ minGap=Math.min(minGap,ds); }
          if (ds<-0.4 && dl<-0.12) overlaps++;
        }
      }
    }
  }
  const ms=Date.now()-t0;
  sim.updateKpi();
  console.log(`[${name}] spawned=${sim.tot.spawned} exited=${sim.tot.exited} inNet=${sim.vehicles.length} vq=${sim.kpi.vq} thr=${sim.kpi.thr.toFixed(0)} avgV=${sim.kpi.avgV.toFixed(1)}km/h delay=${sim.kpi.delay.toFixed(1)}s tt=${sim.kpi.tt.toFixed(1)}s minGap=${minGap.toFixed(2)} overlaps=${overlaps} maxN=${maxN} maxV=${(maxV*3.6).toFixed(0)}km/h ${ms}ms (${(ms/steps).toFixed(3)} ms/step)`);
  console.log('   nodes:', JSON.stringify(sim.nodeTable().map(r=>({id:r.id,sig:r.signal,n:r.n,flow:Math.round(r.flow),delay:+r.delay.toFixed(1),los:r.los,maxQ:Math.round(r.maxQ)}))));
  return sim;
}
run('cross 1-lane signal 300/h', cross(1,true), {flow:300,seed:1}, 1800);
run('cross 1-lane signal 300/h no moto', cross(1,true), {flow:300,seed:1,moto:0}, 1800);
run('cross 1-lane signal 600/h', cross(1,true), {flow:600,seed:2}, 1800);
run('cross 2-lane signal 800/h', cross(2,true), {flow:800,seed:3}, 1800);
run('cross 2-lane signal 1500/h (over)', cross(2,true), {flow:1500,seed:4}, 1800);
run('cross 1-lane unsignal 300/h', cross(1,false), {flow:300,seed:5}, 1800);
run('cross 2-lane unsignal 500/h', cross(2,false), {flow:500,seed:6}, 1800);
// straight road with bottleneck 3 -> 1
const b = RS.createNet();
RS.addSegment(b,-400,0,0,0,{lanes:3,limit:60,oneway:true});
RS.addSegment(b,0,0,400,0,{lanes:1,limit:60,oneway:true});
run('bottleneck 3->1 1500/h', b, {flow:1500,seed:7,truck:10}, 1200);
run('cross 2-lane signal 400/h', cross(2,true), {flow:400,seed:8}, 1800);
run('cross 2-lane signal 600/h', cross(2,true), {flow:600,seed:9}, 1800);
run('cross 1-lane unsignal 500/h', cross(1,false), {flow:500,seed:10}, 1800);

function roundabout(legs, lanes, R){
  const net = RS.createNet();
  for (let q=0;q<legs;q++){ const a=q*2*Math.PI/legs; RS.addSegment(net,0,0,Math.cos(a)*260,Math.sin(a)*260,{lanes,limit:50,oneway:false}); }
  const c = RS.nearNode(net,0,0,5); c.ring = R;
  return net;
}
run('roundabout 4-leg R=14 400/h', roundabout(4,1,14), {flow:400,seed:21}, 1800);
run('roundabout 4-leg R=18 2ln 450/h', roundabout(4,2,18), {flow:450,seed:22}, 1800);
run('roundabout 4-leg R=14 550/h (over)', roundabout(4,1,14), {flow:550,seed:26}, 1800);
run('roundabout 3-leg R=11 350/h', roundabout(3,1,11), {flow:350,seed:23}, 1800);
run('moto heavy 2-lane signal 500/h', cross(2,true), {flow:500,seed:24,moto:50}, 1800);
run('moto heavy no hook', cross(2,true), {flow:500,seed:25,moto:50,hook:false}, 1800);
