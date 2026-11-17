const RS=require('../src/sim.js');
function cross(lanes,sig){ const net=RS.createNet();
 RS.addSegment(net,-250,0,250,0,{lanes,limit:50}); RS.addSegment(net,0,-250,0,250,{lanes,limit:50});
 for(const n of net.nodes) if(RS.nodeDegree(net,n.id)===4&&sig) n.sig=RS.defaultSig(); return net; }
function run(name,net,opt,secs){
  const sim=new RS.Sim(net,opt); let hooks=0, waits=0, maxWait=0, released=0, types={}, maxBox=0;
  const prev=new Map();
  for(let i=0;i<secs*10;i++){
    for(const v of sim.vehicles) prev.set(v.id, v.hookWait);
    sim.step();
    for(const v of sim.vehicles){ if(v.hookWait&&!prev.get(v.id)) waits++; if(prev.get(v.id)&&!v.hookWait&&!v.conn) released++;
      if(v.hookWait) maxWait=Math.max(maxWait, sim.t-v.hookT); }
    for(const gn of sim.g.nodes.values()) for(const k in gn.box) maxBox=Math.max(maxBox,gn.box[k]);
  }
  for(const v of sim.vehicles) types[v.type]=(types[v.type]||0)+1;
  const stuck=sim.vehicles.filter(v=>v.hookWait&&sim.t-v.hookT>200).length;
  sim.updateKpi();
  console.log(`[${name}] 待轉進入=${waits} 完成=${released} 最久等待=${maxWait.toFixed(0)}s 卡住>200s=${stuck} 待轉區最多=${maxBox} 在途車種=${JSON.stringify(types)} exited=${sim.tot.exited} avgV=${sim.kpi.avgV.toFixed(1)}`);
}
run('兩段式左轉 1車道 300/h moto50%', cross(1,true), {flow:300,seed:31,moto:50}, 1800);
run('兩段式左轉 2車道 500/h moto40%', cross(2,true), {flow:500,seed:32,moto:40}, 1800);
run('兩段式左轉 2車道 800/h moto60%', cross(2,true), {flow:800,seed:33,moto:60}, 1800);
run('關閉兩段式 2車道 500/h moto40%', cross(2,true), {flow:500,seed:34,moto:40,hook:false}, 1800);
