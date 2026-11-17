const RS=require('../src/sim.js');
function cross(lanes,sig){ const net=RS.createNet();
 RS.addSegment(net,-250,0,250,0,{lanes,limit:50}); RS.addSegment(net,0,-250,0,250,{lanes,limit:50});
 for(const n of net.nodes) if(RS.nodeDegree(net,n.id)===4&&sig) n.sig=Object.assign(RS.defaultSig(),sig); return net; }
function run(name,net,opt,secs){
  const sim=new RS.Sim(net,opt); const prev=new Map(); let viol=0, entries=0, rightRed=0, byTurn={L:0,S:0,R:0}, maxOcc=0, bad=[], minGap=99;
  for(let i=0;i<secs*10;i++){
    for(const v of sim.vehicles) prev.set(v.id,!!v.conn);
    sim.step();
    if(i%10===0) for(const e of sim.g.edges) for(const L of e.lanesArr){ const a=L.slice().sort((p,q)=>p.s-q.s); for(let k=0;k+1<a.length;k++){ const g=a[k+1].s-a[k+1].len-a[k].s; if(g<minGap) minGap=g; } }
    for(const v of sim.vehicles){ if(v.conn && prev.get(v.id)===false){ entries++; byTurn[v.conn.turn]++;
        const gn=v.conn.e.b; if(gn.sig){ const st=RS.sigState(gn.sig,v.conn.e.group,v.conn.turn,sim.t-RS.DT); const st2=RS.sigState(gn.sig,v.conn.e.group,v.conn.turn,sim.t);
          if(st==='R'&&st2==='R'){ viol++; if(v.conn.turn==='R') rightRed++; if(bad.length<3) bad.push({t:sim.t.toFixed(1),turn:v.conn.turn,v:v.v.toFixed(1)}); } } } }
  }
  sim.updateKpi(); const k=sim.kpi;
  console.log(`[${name}] entries=${entries} redViolations=${viol} (rightOnRed=${rightRed}) turns=${JSON.stringify(byTurn)} avgV=${k.avgV.toFixed(1)} delay=${k.delay.toFixed(1)} vq=${k.vq} exited=${sim.tot.exited} minGap=${minGap.toFixed(2)}`, bad.length?JSON.stringify(bad):'');
}
run('1-lane sig 400',cross(1,{}),{flow:400,seed:3},1200);
run('2-lane sig 500',cross(2,{}),{flow:500,seed:4},1200);
run('2-lane sig 700',cross(2,{}),{flow:700,seed:5},1200);
run('2-lane left-phase 500',cross(2,{lA:12,lB:12}),{flow:500,seed:6},1200);
run('3-lane L+R phases 600',cross(3,{lA:10,lB:10,rA:8,rB:8}),{flow:600,seed:7},1200);
run('1-lane L phase 300',cross(1,{lA:10,lB:10}),{flow:300,seed:8},1200);
run('3-lane L+R phases 300',cross(3,{lA:10,lB:10,rA:8,rB:8}),{flow:300,seed:9},1200);
run('2-lane L phase 300',cross(2,{lA:10,lB:10}),{flow:300,seed:10},1200);
