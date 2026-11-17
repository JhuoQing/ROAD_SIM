import re
t=open('src/template.html').read()
t=t.replace('/*CSS*/',open('src/style.css').read()).replace('/*SIM*/',open('src/sim.js').read().replace('</script','<\\/script')).replace('/*APP*/',open('src/app.js').read().replace('</script','<\\/script'))
open('index.html','w').write(t)
print(len(t))
