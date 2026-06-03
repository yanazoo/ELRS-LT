'use strict';

function isOnline(uid, now){
  if(!uid)return false;
  var sr=scanResults[uid.toUpperCase()]||scanResults[uid];
  if(!sr)return false;
  return sr.receivedAt && (now - sr.receivedAt < 30000);
}

function renderRoster(){
  Object.keys(scanResults).forEach(mac => {
    var macId = mac.replace(/:/g, '');
    var el = document.getElementById('scanName-' + macId);
    if (el && el.value.trim()) scanResults[mac].inputName = el.value;
  });

  var filter=(document.getElementById('searchInput').value||'').toLowerCase().trim();
  var list=document.getElementById('rosterList');
  var badge=document.getElementById('rosterCountBadge');
  badge.textContent='('+rosterData.length+'/20)';

  var filtered=rosterData.filter(function(r){
    if(!filter)return true;
    return r.name.toLowerCase().includes(filter)||(r.uid||'').toLowerCase().includes(filter);
  });

  var now=Date.now();
  filtered.sort(function(a,b){
    var aOn=isOnline(a.uid,now),bOn=isOnline(b.uid,now);
    if(aOn!==bOn) return aOn?-1:1;
    var aCh=a.activeSlot>=0?a.activeSlot:99;
    var bCh=b.activeSlot>=0?b.activeSlot:99;
    if(aCh!==bCh) return aCh-bCh;
    return a.id-b.id;
  });

  if(!filtered.length){
    list.innerHTML='<div style="padding:20px;text-align:center;color:var(--muted);font-size:13px">'+(rosterData.length?'該当なし':'パイロット未登録')+'</div>';
    return;
  }

  list.innerHTML='';
  filtered.forEach(function(r){
    var isEditing=(editingRosterId===r.id);
    var item=document.createElement('div');item.className='roster-item';item.id='ri-'+r.id;
    var onlineMark=isOnline(r.uid,now)?'<span class="online-badge">📶 オンライン</span>':'';
    var activeBadge='';
    if(r.activeSlot>=0)activeBadge='<span class="active-badge '+PCLS[r.activeSlot]+'">Ch'+(r.activeSlot+1)+'</span>';

    if(isEditing){
      var enterVal=(r.enter!=null?r.enter:-80);
      var exitVal=(r.exit!=null?r.exit:-90);
      var editSlot=activeSlotsLocal.indexOf(r.id);
      var editEp1=editSlot>=0?slotEp1Macs[editSlot]:'';
      var ep1Hint=editEp1?'<span style="color:var(--ok);font-size:9px">→ Ch'+(editSlot+1)+' EP1 自動書込</span>':'';
      var sid=r.id;
      item.innerHTML=
        '<div style="flex:1;min-width:160px;display:flex;flex-direction:column;gap:4px">'
          +'<input type="text" id="editName" value="'+esc(r.name)+'" maxlength="20" placeholder="パイロット名" autocomplete="off" oninput="scheduleEditSave('+sid+')" style="background:var(--bg);border:1px solid var(--accent);color:var(--tx);border-radius:5px;padding:3px 7px;font-size:12px;width:100%">'
          +'<input type="text" id="editYomi" value="'+esc(r.yomi||'')+'" maxlength="20" placeholder="読み方（よみかた）" autocomplete="off" oninput="scheduleEditSave('+sid+')" style="background:var(--bg);border:1px solid var(--bd);color:var(--tx);border-radius:5px;padding:3px 7px;font-size:11px;width:100%">'
        +'</div>'
        +'<div style="display:flex;flex-direction:column;gap:2px">'
          +'<input type="text" id="editPhrase" placeholder="バインドフレーズ" autocomplete="off" oninput="onEditPhraseInput();scheduleEditSave('+sid+')" style="width:130px;background:var(--bg);border:1px solid var(--bd);color:var(--muted);border-radius:5px;padding:2px 7px;font-size:10px" title="入力でUIDを自動計算">'
          +'<input type="text" id="editUid" value="'+esc(r.uid||'')+'" maxlength="17" placeholder="AA:BB:CC:DD:EE:FF" autocomplete="off" oninput="scheduleEditSave('+sid+')" style="width:130px;font-family:monospace;background:var(--bg);border:1px solid var(--bd);color:var(--tx);border-radius:5px;padding:3px 7px;font-size:12px">'
          +ep1Hint
        +'</div>'
        +'<div style="display:flex;flex-direction:column;gap:2px">'
          +'<div style="display:flex;align-items:center;gap:3px">'
            +'<label style="color:var(--muted);font-size:10px;width:20px">入</label>'
            +'<input type="number" id="editEnter" value="'+enterVal+'" min="-120" max="0" oninput="scheduleEditSave('+sid+')" style="width:58px;background:var(--bg);border:1px solid var(--bd);color:var(--tx);border-radius:5px;padding:2px 4px;font-size:11px;text-align:center">'
            +'<span style="color:var(--muted);font-size:10px">dBm</span>'
          +'</div>'
          +'<div style="display:flex;align-items:center;gap:3px">'
            +'<label style="color:var(--muted);font-size:10px;width:20px">出</label>'
            +'<input type="number" id="editExit" value="'+exitVal+'" min="-120" max="0" oninput="scheduleEditSave('+sid+')" style="width:58px;background:var(--bg);border:1px solid var(--bd);color:var(--tx);border-radius:5px;padding:2px 4px;font-size:11px;text-align:center">'
            +'<span style="color:var(--muted);font-size:10px">dBm</span>'
          +'</div>'
        +'</div>'
        +'<div class="roster-actions" style="flex-direction:column;align-items:flex-end;gap:2px">'
          +'<button onclick="cancelEdit()" class="btn-secondary" style="font-size:11px;padding:3px 9px">×</button>'
          +'<span id="editSaveHint" style="font-size:9px;color:var(--muted);white-space:nowrap"></span>'
        +'</div>';
    } else {
      var chOpts='<option value="-1"'+(r.activeSlot<0?' selected':'')+'>未割当</option>';
      for(var ch=0;ch<N;ch++){
        chOpts+='<option value="'+ch+'"'+(r.activeSlot===ch?' selected':'')+'>Ch'+(ch+1)+'</option>';
      }
      var yomiLine=r.yomi?'<span class="roster-yomi">'+esc(r.yomi)+'</span>':'';
      item.innerHTML=
        '<div style="flex:1;min-width:80px">'
          +'<span class="roster-name">'+esc(r.name)+yomiLine+'</span>'
        +'</div>'
        +'<span class="roster-uid">'+(r.uid||'<span style="color:var(--err)">UID未設定</span>')+'</span>'
        +onlineMark+activeBadge
        +'<div class="roster-actions">'
          +(raceRunning
            ?'<span style="color:var(--muted);font-size:10px">計測中</span>'
            :'<select class="ch-select" data-rid="'+r.id+'" onchange="onChSelectChange(this)">'+chOpts+'</select>'
              +'<button onclick="startEdit('+r.id+')" class="btn-secondary" style="font-size:11px;padding:3px 8px">編集</button>'
          )
          +'<button onclick="deleteRosterPilot('+r.id+')" class="btn-danger" style="font-size:11px;padding:3px 8px"'+(raceRunning?' disabled':'')+'>削除</button>'
        +'</div>';
    }
    list.appendChild(item);
  });
}

async function onChSelectChange(sel){
  if(raceRunning){toast('⚠️ 計測中はチャンネル変更できません');return;}
  var rid=parseInt(sel.dataset.rid);
  var newCh=parseInt(sel.value);
  var newSlots=activeSlotsLocal.slice();
  for(var i=0;i<N;i++) if(newSlots[i]===rid) newSlots[i]=-1;
  if(newCh>=0){
    for(var i=0;i<N;i++) if(newSlots[i]!==rid&&newSlots[i]===newSlots[newCh])newSlots[i]=-1;
    newSlots[newCh]=rid;
  }
  try{
    var r=await fetch('/api/active',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({slots:newSlots})});
    if(r.ok){
      activeSlotsLocal=newSlots;applyActiveToSlots();buildRaceCards();buildCalibCards();await loadRoster();
      if(newCh>=0&&slotEp1Macs[newCh])toast('✓ Ch'+(newCh+1)+' 割当 + EP1 へ即時プロビジョニング済み');
    }
    else toast('⚠️ 保存エラー');
  }catch(e){toast('⚠️ 接続エラー');}
}

var EP1_STATE_NAMES=['待機中(未設定)','スキャン中','追跡中'];

function updateEp1List(){
  var el=document.getElementById('ep1List');
  var sel=document.getElementById('addEp1Mac');
  var now=Date.now();
  var macs=Object.keys(ep1Nodes).filter(function(m){return now-ep1Nodes[m].lastSeenAt<30000;});

  // Update addForm EP1 selector
  if(sel){
    var prev=sel.value;
    sel.innerHTML='<option value="">なし</option>'
      +macs.map(function(mac,i){
        var stName=EP1_STATE_NAMES[ep1Nodes[mac].state]||'不明';
        return '<option value="'+esc(mac)+'"'+(prev===mac?' selected':'')+'>EP1 #'+(i+1)+' — '+esc(stName)+'</option>';
      }).join('');
  }

  if(!el)return;
  if(!macs.length){
    el.innerHTML='<span style="color:var(--muted);font-size:12px">EP1 ノードを待機中... （EP1の電源を入れてください）</span>';
    return;
  }
  el.innerHTML=macs.map(function(mac,i){
    var n=ep1Nodes[mac];
    var stName=EP1_STATE_NAMES[n.state]||'不明';
    var stColor=(n.state===2)?'var(--ok)':(n.state===1)?'var(--accent)':'var(--muted)';
    var uidLine=n.uid?'<span style="font-family:monospace;font-size:10px;color:var(--muted)"> UID:'+esc(n.uid)+'</span>':'';
    var curSlot=slotEp1Macs.indexOf(mac);
    var slotBadge=curSlot>=0
      ?'<span style="margin-left:auto;padding:2px 8px;border-radius:4px;font-size:11px;font-weight:700;background:var(--'+PCLS[curSlot]+'-bg);color:var(--'+PCLS[curSlot]+');border:1px solid var(--'+PCLS[curSlot]+')">'+'Ch'+(curSlot+1)+'</span>'
      :'<span style="margin-left:auto;font-size:10px;color:var(--muted)">自動割当待ち</span>';
    return '<div style="display:flex;align-items:center;gap:8px;padding:4px 0;flex-wrap:wrap">'
      +'<span style="font-weight:700;color:var(--accent);font-size:13px">#'+(i+1)+'</span>'
      +'<span style="font-family:monospace;font-size:12px">'+esc(mac)+'</span>'
      +'<span style="font-size:11px;color:'+stColor+'">'+esc(stName)+'</span>'
      +uidLine
      +slotBadge
      +'</div>';
  }).join('');
}

async function onEp1SlotChange(mac,selEl){
  if(raceRunning){toast('⚠️ 計測中は EP1 ノードの割当変更できません');return;}
  var newSlot=parseInt(selEl.value);
  var arr=slotEp1Macs.slice();
  for(var i=0;i<N;i++)if(arr[i]===mac)arr[i]='';
  if(newSlot>=0&&newSlot<N)arr[newSlot]=mac;
  slotEp1Macs=arr;
  try{
    await fetch('/api/ep1/slots',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({slots:slotEp1Macs})});
    toast(newSlot>=0?'✓ EP1 を Ch'+(newSlot+1)+' に割当しました':'✓ EP1 の割当を解除しました');
  }catch(e){toast('⚠️ 接続エラー');}
}

async function loadEp1Slots(){
  try{
    var r=await fetch('/api/ep1/slots');
    if(r.ok){
      var d=await r.json();
      slotEp1Macs=(d.slots||['','','','']).map(function(s){return s||'';});
      updateEp1List();
    }
  }catch(e){}
}

function onAddPhraseInput(){
  var phrase=document.getElementById('addPhrase').value;
  if(!phrase)return;
  if(typeof phraseToUid!=='function')return;
  var uid=phraseToUid(phrase);
  var el=document.getElementById('addUid');
  if(el)el.value=uid;
}

function onEditPhraseInput(){
  var phrase=document.getElementById('editPhrase').value;
  if(!phrase||typeof phraseToUid!=='function')return;
  var uid=phraseToUid(phrase);
  var el=document.getElementById('editUid');
  if(el)el.value=uid;
}

function showAddForm(){document.getElementById('addForm').style.display='block';updateEp1List();document.getElementById('addName').focus();}
function hideAddForm(){
  document.getElementById('addForm').style.display='none';
  document.getElementById('addName').value='';
  document.getElementById('addYomi').value='';
  document.getElementById('addPhrase').value='';
  document.getElementById('addUid').value='';
  document.getElementById('addEp1Mac').value='';
  document.getElementById('addEnter').value='-80';
  document.getElementById('addExit').value='-90';
}

async function submitAdd(){
  var name=document.getElementById('addName').value.trim();
  var yomi=document.getElementById('addYomi').value.trim();
  var uid=document.getElementById('addUid').value.trim().toUpperCase();
  var ep1Mac=document.getElementById('addEp1Mac').value;
  var enter=parseInt(document.getElementById('addEnter').value)||(-80);
  var exit_=parseInt(document.getElementById('addExit').value)||(-90);
  if(!name){toast('⚠️ 名前を入力してください');return;}
  var validUid=/^[0-9A-F]{2}(:[0-9A-F]{2}){5}$/.test(uid);
  if(uid&&!validUid){toast('⚠️ UID形式: AA:BB:CC:DD:EE:FF');return;}
  try{
    var r=await fetch('/api/pilots',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name,yomi,uid:validUid?uid:''})});
    if(r.ok){
      var body=await r.json();
      await fetch('/api/calib',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id:body.id,enter,exit:exit_})});
      if(ep1Mac&&validUid){
        await fetch('/api/ep1/provision',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mac:ep1Mac,uid:uid})});
        await loadRoster();hideAddForm();toast('✓ '+name+' を登録・EP1 へプロビジョニングしました');
      }else{
        await loadRoster();hideAddForm();toast('✓ '+name+' を登録しました');
      }
    }
    else toast('⚠️ 登録エラー');
  }catch(e){toast('⚠️ 接続エラー');}
}

function startEdit(id){editingRosterId=id;renderRoster();setTimeout(()=>{var el=document.getElementById('editName');if(el)el.focus();},0);}
function cancelEdit(){clearTimeout(_editSaveTimer);editingRosterId=null;renderRoster();}

var _editSaveTimer=null;
function scheduleEditSave(id){
  clearTimeout(_editSaveTimer);
  var h=document.getElementById('editSaveHint');
  if(h){h.textContent='編集中...';h.style.color='var(--muted)';}
  _editSaveTimer=setTimeout(function(){doAutoSaveEdit(id);},800);
}

async function doAutoSaveEdit(id){
  if(raceRunning){
    var h=document.getElementById('editSaveHint');
    if(h){h.textContent='⚠ 計測中は保存不可';h.style.color='var(--err)';}
    return;
  }
  var nameEl=document.getElementById('editName');
  if(!nameEl)return;
  var name=nameEl.value.trim();
  var yomi=(document.getElementById('editYomi')||{}).value||'';
  yomi=yomi.trim();
  var uid=((document.getElementById('editUid')||{}).value||'').trim().toUpperCase();
  var enter=parseInt((document.getElementById('editEnter')||{}).value)||(-80);
  var exit_=parseInt((document.getElementById('editExit')||{}).value)||(-90);
  if(!name)return;
  var validUid=/^[0-9A-F]{2}(:[0-9A-F]{2}){5}$/.test(uid);
  if(uid&&!validUid){
    var h=document.getElementById('editSaveHint');
    if(h){h.textContent='⚠ UID形式エラー';h.style.color='var(--err)';}
    return;
  }
  var slot=activeSlotsLocal.indexOf(id);
  var ep1Mac=slot>=0?slotEp1Macs[slot]:'';
  try{
    var r=await fetch('/api/pilots',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id,name,yomi,uid:validUid?uid:''})});
    if(!r.ok){
      var h=document.getElementById('editSaveHint');
      if(h){h.textContent='⚠ 保存失敗';h.style.color='var(--err)';}
      return;
    }
    await fetch('/api/calib',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id,enter,exit:exit_})});
    var h=document.getElementById('editSaveHint');
    if(h){h.textContent='✓ 保存済み';h.style.color='var(--ok)';}
    if(ep1Mac&&validUid)toast('✓ Ch'+(slot+1)+' EP1 へプロビジョニング済み',1500);
  }catch(e){
    var h=document.getElementById('editSaveHint');
    if(h){h.textContent='⚠ 保存失敗';h.style.color='var(--err)';}
  }
}

async function deleteRosterPilot(id){
  var rowEl=document.getElementById('ri-'+id);
  if(rowEl)rowEl.remove();
  if(editingRosterId===id)editingRosterId=null;
  rosterData=rosterData.filter(x=>x.id!==id);
  rebuildRosterIndex();
  var badge=document.getElementById('rosterCountBadge');
  if(badge)badge.textContent='('+rosterData.length+'/20)';
  try{
    var res=await fetch('/api/pilots/delete',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id})});
    if(res.ok){await loadRoster();applyActiveToSlots();buildRaceCards();buildCalibCards();toast('削除しました');}
    else{await loadRoster();toast('⚠️ 削除エラー');}
  }catch(e){await loadRoster();toast('⚠️ 接続エラー');}
}

function updateScanList(){
  var _af=document.activeElement;
  if(_af&&_af.id&&(_af.id.startsWith('scanName-')||_af.id.startsWith('scanYomi-')||_af.id.startsWith('scanEnter-')||_af.id.startsWith('scanExit-')))return;
  var el=document.getElementById('scanList');if(!el)return;
  var macs=Object.keys(scanResults).filter(function(mac){
    var s=scanResults[mac];
    return !(s.assignedRosterId!==undefined&&s.assignedRosterId>=0);
  });
  if(!macs.length){el.innerHTML='<span style="color:var(--muted);font-size:12px">スキャン待機中... 機体の電源を入れてください</span>';return;}
  el.innerHTML=macs.map(function(mac){
    var s=scanResults[mac];
    var done=false;
    var macId=mac.replace(/:/g,'');
    var savedName=s.inputName||s.pilotName||'';
    var rssi=typeof s.rssi==='number'?s.rssi:-80;
    var enterDef=Math.max(-120,Math.min(-1,rssi-5));
    var exitDef=Math.max(-120,Math.min(-1,enterDef-5));
    return '<div id="scan-'+macId+'" style="background:var(--sf2);border:1px solid var(--bd);border-radius:8px;padding:10px 12px;margin-bottom:8px">'
      +'<div style="display:flex;align-items:center;gap:8px;margin-bottom:8px">'
      +  '<span style="font-family:monospace;font-size:13px;color:var(--accent);flex:1">'+mac+'</span>'
      +  '<span style="color:var(--muted);font-size:11px">'+s.rssi+' dBm</span>'
      +  (done?'<span style="color:var(--ok);font-size:11px;font-weight:700">✓ 登録済み</span>':'')
      +'</div>'
      +'<div style="display:flex;flex-direction:column;gap:4px;margin-bottom:6px">'
      +  '<input type="text" id="scanName-'+macId+'" placeholder="パイロット名" maxlength="20" value="'+esc(savedName)+'" autocomplete="off"'
      +    ' style="background:var(--bg);border:1px solid var(--bd);color:var(--tx);border-radius:6px;padding:5px 8px;font-size:13px"'
      +    (done?' disabled':'')+' onkeydown="if(event.key===\'Enter\')registerScanPilot(\''+mac+'\')">'
      +  '<input type="text" id="scanYomi-'+macId+'" placeholder="よみかた（TTS読み上げ用）" maxlength="20" autocomplete="off"'
      +    ' style="background:var(--bg);border:1px solid var(--bd);color:var(--tx);border-radius:6px;padding:4px 8px;font-size:11px"'
      +    (done?' disabled':'')+'>'
      +'</div>'
      +'<div style="display:flex;gap:5px;align-items:center">'
      +  '<span style="font-size:11px;color:var(--muted)">入</span>'
      +  '<input type="number" id="scanEnter-'+macId+'" value="'+(done?-80:enterDef)+'" min="-120" max="-1"'+(done?' disabled':'')
      +    ' style="width:62px;background:var(--bg);border:1px solid var(--bd);color:var(--tx);border-radius:6px;padding:4px 5px;font-size:12px;text-align:center">'
      +  '<span style="font-size:10px;color:var(--muted)">dBm</span>'
      +  '<span style="font-size:11px;color:var(--muted);margin-left:4px">出</span>'
      +  '<input type="number" id="scanExit-'+macId+'" value="'+(done?-90:exitDef)+'" min="-120" max="-1"'+(done?' disabled':'')
      +    ' style="width:62px;background:var(--bg);border:1px solid var(--bd);color:var(--tx);border-radius:6px;padding:4px 5px;font-size:12px;text-align:center">'
      +  '<span style="font-size:10px;color:var(--muted)">dBm</span>'
      +  '<button id="scanBtn-'+macId+'" onclick="registerScanPilot(\''+mac+'\')" class="btn-success" style="font-size:12px;padding:5px 10px;white-space:nowrap;margin-left:auto"'+(done?' disabled':'')+'>パイロット情報に追加</button>'
      +'</div>'
      +'</div>';
  }).join('');
}

async function registerScanPilot(mac){
  var macId=mac.replace(/:/g,'');
  var nameEl=document.getElementById('scanName-'+macId);
  if(!nameEl)return;
  var name=nameEl.value.trim();
  if(!name){toast('⚠️ 名前を入力してください');return;}
  var yomiEl=document.getElementById('scanYomi-'+macId);
  var yomi=yomiEl?yomiEl.value.trim():'';
  var enterEl=document.getElementById('scanEnter-'+macId);
  var exitEl=document.getElementById('scanExit-'+macId);
  var enter=enterEl?parseInt(enterEl.value)||(-80):-80;
  var exit_=exitEl?parseInt(exitEl.value)||(-90):-90;
  try{
    var r=await fetch('/api/pilots',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name,yomi,uid:mac})});
    if(!r.ok){toast('⚠️ 登録エラー');return;}
    var body=await r.json();
    await fetch('/api/calib',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id:body.id,enter,exit:exit_})});
    scanResults[mac].assignedRosterId=body.id;
    scanResults[mac].pilotName=name;
    scanResults[mac].inputName='';
    await loadRoster();updateScanList();
    toast('✓ '+name+' をパイロット情報に追加しました');
  }catch(e){toast('⚠️ 接続エラー');}
}

async function scanRefresh(showFeedback){
  try{
    await fetch('/api/scan/refresh',{method:'POST'});
    if(showFeedback)toast('🔄 スキャン更新を送信しました');
  }catch(e){if(showFeedback)toast('⚠️ 接続エラー');}
}

function clearScan(){scanResults={};updateScanList();fetch('/api/scan/clear',{method:'POST'}).catch(()=>{});}

async function autoAssignChannels(){
  var now=Date.now();
  var online=rosterData
    .filter(r=>r.uid&&isOnline(r.uid,now))
    .sort((a,b)=>{
      var sa=(scanResults[a.uid.toUpperCase()]||scanResults[a.uid]||{});
      var sb=(scanResults[b.uid.toUpperCase()]||scanResults[b.uid]||{});
      return (sa.firstSeenAt||9999999999999)-(sb.firstSeenAt||9999999999999);
    });
  if(!online.length){toast('⚠️ オンラインの機体がありません');return;}
  var newSlots=[-1,-1,-1,-1];
  for(var i=0;i<Math.min(online.length,N);i++)newSlots[i]=online[i].id;
  try{
    var r=await fetch('/api/active',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({slots:newSlots})});
    if(r.ok){
      activeSlotsLocal=newSlots;applyActiveToSlots();buildRaceCards();buildCalibCards();
      await loadRoster();
      toast('✅ '+Math.min(online.length,N)+'機を電源ON順でCh割当しました');
    }else toast('⚠️ 割当エラー');
  }catch(e){toast('⚠️ 接続エラー');}
}

async function clearChannelAssignments(){
  var newSlots=[-1,-1,-1,-1];
  try{
    var r=await fetch('/api/active',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({slots:newSlots})});
    if(r.ok){
      activeSlotsLocal=newSlots;applyActiveToSlots();buildRaceCards();buildCalibCards();
      await loadRoster();toast('✅ チャンネル割当を全解除しました');
    }else toast('⚠️ エラー');
  }catch(e){toast('⚠️ 接続エラー');}
}

async function sdBackup(){
  try{
    var r=await fetch('/api/sd/pilots/backup',{method:'POST'});
    if(r.ok)toast('✅ SDカードにバックアップしました');
    else toast('⚠️ バックアップエラー: '+(await r.text()));
  }catch(e){toast('⚠️ 接続エラー');}
}
async function sdRestore(){
  if(!confirm('SDカードからパイロット情報を復元します。現在のデータは上書きされます。よろしいですか？'))return;
  try{
    var r=await fetch('/api/sd/pilots/restore',{method:'POST'});
    if(r.ok)toast('📤 復元要求を送信しました。完了まで少々お待ちください…',4000);
    else toast('⚠️ 復元エラー');
  }catch(e){toast('⚠️ 接続エラー');}
}

function saveGlobalConfig(){
  announceMode=document.getElementById('announceMode').value;
  speechRate=parseFloat(document.getElementById('speechRateN').value)||1.1;
  lapMode=document.getElementById('lapModeSelect').value;
  var sdEl=document.getElementById('sdLogModeSelect');
  if(sdEl)sdLogMode=sdEl.value;
  var cdSec=parseFloat(document.getElementById('cooldownInput').value)||3.0;
  cooldownMs=Math.max(500,Math.min(30000,Math.round(cdSec*1000)));
  document.getElementById('cooldownInput').value=(cooldownMs/1000).toFixed(1);
  localStorage.setItem('announce',announceMode);
  localStorage.setItem('srate',String(speechRate));
  localStorage.setItem('lapMode',lapMode);
  localStorage.setItem('cooldownMs',String(cooldownMs));
  localStorage.setItem('sdLogMode',sdLogMode);
  fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({lapMode:lapMode==='immediate'?1:0,cooldownMs:cooldownMs,sdLogMode:sdLogModeInt(sdLogMode)})}).catch(()=>{});
}

function updateSdSection(present){
  sdPresent=present;
  var sec=document.getElementById('sdSection');
  if(sec)sec.style.display=present?'block':'none';
  var st=document.getElementById('sdTabStatus');
  if(st)st.innerHTML=present
    ?'<p style="color:var(--ok);font-size:12px">✅ SDカード検出済み — レース中はダウンロードしないでください</p>'
    :'<p style="color:var(--err);font-size:12px">⚠ SDカードが見つかりません</p>';
  if(!present){
    var wrap=document.getElementById('sdFileListWrap');
    if(wrap)wrap.innerHTML='<p style="color:var(--muted);font-size:12px">SDカードが挿入されていません</p>';
  }
}
