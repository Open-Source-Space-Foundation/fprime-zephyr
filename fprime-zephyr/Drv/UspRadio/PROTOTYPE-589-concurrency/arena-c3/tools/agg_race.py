import glob,re,sys
for f in sorted(glob.glob(sys.argv[1])):
    agg={}; n=0; seeds_lost=0; seeds_bad=0
    for l in open(f):
        if l.startswith('#'): continue
        n+=1
        d=dict((k,int(v)) for k,v in re.findall(r'(\w+)=(\d+)',l.split('|')[1]))
        for k,v in d.items(): agg[k]=agg.get(k,0)+v
        seeds_lost += d.get('lost_asleep',0)>0
        seeds_bad += (d.get('panics',0)>0) or ('live_rx=6/6' not in l)
    wedges=seeds_bad
    print(f.split('_')[-1][:-4], 'seeds',n, 'seeds_with_lost_cmd',seeds_lost,'seeds_with_panic_or_dead_rx',seeds_bad, {k:agg[k] for k in ('panics','config_failed','chg_deferred','send_failed','lost_asleep','tx_sent') if k in agg})

for f in sorted(glob.glob(sys.argv[1])):
    ops=0; wedges=0; n=0
    for l in open(f):
        if l.startswith('#'): continue
        d=dict((k,int(v)) for k,v in re.findall(r'(\w+)=(\d+)',l.split('|')[1]))
        n+=1
        ops+=d.get('ops_before_stall',0)
        wedges+= (d.get('panics',0)>0) or ('live_rx=6/6' not in l)
    print('   ', f.split('_')[-1][:-4], 'ops_executed_until_stall_or_end', ops, 'stalls', wedges, ('-> 1 stall per %.0f ops' % (ops/wedges)) if wedges else '-> no stall observed in %d ops' % ops)
