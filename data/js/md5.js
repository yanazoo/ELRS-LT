/* md5.js — RFC 1321 MD5, public domain (based on Joseph Myers).
 * Exposes window.phraseToUid(phrase) -> "AA:BB:CC:DD:EE:FF" (first 6 bytes of MD5).
 * Matches: Python hashlib.md5(phrase.encode('utf-8')).digest()[:6]
 * Test vectors:
 *   "yanazoo" -> 45:7C:D7:AE:EB:B1
 *   "test"    -> 09:8F:6B:CD:46:21
 *   "hello"   -> 5D:41:40:2A:BC:4B */
(function () {
'use strict';

function safeAdd(x, y) {
  var l = (x & 0xffff) + (y & 0xffff);
  return (((x >> 16) + (y >> 16) + (l >> 16)) << 16) | (l & 0xffff);
}
function rol(n, s) { return (n << s) | (n >>> (32 - s)); }
function cmn(q, a, b, x, s, t) { return safeAdd(rol(safeAdd(safeAdd(a, q), safeAdd(x, t)), s), b); }
function ff(a,b,c,d,x,s,t){return cmn((b&c)|(~b&d),a,b,x,s,t)}
function gg(a,b,c,d,x,s,t){return cmn((b&d)|(c&~d),a,b,x,s,t)}
function hh(a,b,c,d,x,s,t){return cmn(b^c^d,a,b,x,s,t)}
function ii(a,b,c,d,x,s,t){return cmn(c^(b|~d),a,b,x,s,t)}

function md5block(s, M) {
  var a=s[0],b=s[1],c=s[2],d=s[3];
  a=ff(a,b,c,d,M[0],7,-680876936);  d=ff(d,a,b,c,M[1],12,-389564586);  c=ff(c,d,a,b,M[2],17,606105819);   b=ff(b,c,d,a,M[3],22,-1044525330);
  a=ff(a,b,c,d,M[4],7,-176418897);  d=ff(d,a,b,c,M[5],12,1200080426);  c=ff(c,d,a,b,M[6],17,-1473231341); b=ff(b,c,d,a,M[7],22,-45705983);
  a=ff(a,b,c,d,M[8],7,1770035416);  d=ff(d,a,b,c,M[9],12,-1958414417); c=ff(c,d,a,b,M[10],17,-42063);     b=ff(b,c,d,a,M[11],22,-1990404162);
  a=ff(a,b,c,d,M[12],7,1804603682); d=ff(d,a,b,c,M[13],12,-40341101);  c=ff(c,d,a,b,M[14],17,-1502002290);b=ff(b,c,d,a,M[15],22,1236535329);
  a=gg(a,b,c,d,M[1],5,-165796510);  d=gg(d,a,b,c,M[6],9,-1069501632);  c=gg(c,d,a,b,M[11],14,643717713); b=gg(b,c,d,a,M[0],20,-373897302);
  a=gg(a,b,c,d,M[5],5,-701558691);  d=gg(d,a,b,c,M[10],9,38016083);    c=gg(c,d,a,b,M[15],14,-660478335);b=gg(b,c,d,a,M[4],20,-405537848);
  a=gg(a,b,c,d,M[9],5,568446438);   d=gg(d,a,b,c,M[14],9,-1019803690); c=gg(c,d,a,b,M[3],14,-187363961); b=gg(b,c,d,a,M[8],20,1163531501);
  a=gg(a,b,c,d,M[13],5,-1444681467);d=gg(d,a,b,c,M[2],9,-51403784);    c=gg(c,d,a,b,M[7],14,1735328473); b=gg(b,c,d,a,M[12],20,-1926607734);
  a=hh(a,b,c,d,M[5],4,-378558);     d=hh(d,a,b,c,M[8],11,-2022574463); c=hh(c,d,a,b,M[11],16,1839030562);b=hh(b,c,d,a,M[14],23,-35309556);
  a=hh(a,b,c,d,M[1],4,-1530992060);d=hh(d,a,b,c,M[4],11,1272893353);  c=hh(c,d,a,b,M[7],16,-155497632); b=hh(b,c,d,a,M[10],23,-1094730640);
  a=hh(a,b,c,d,M[13],4,681279174);  d=hh(d,a,b,c,M[0],11,-358537222);  c=hh(c,d,a,b,M[3],16,-722521979); b=hh(b,c,d,a,M[6],23,76029189);
  a=hh(a,b,c,d,M[9],4,-640364487);  d=hh(d,a,b,c,M[12],11,-421815835); c=hh(c,d,a,b,M[15],16,530742520); b=hh(b,c,d,a,M[2],23,-995338651);
  a=ii(a,b,c,d,M[0],6,-198630844);  d=ii(d,a,b,c,M[7],10,1126891415);  c=ii(c,d,a,b,M[14],15,-1416354905);b=ii(b,c,d,a,M[5],21,-57434055);
  a=ii(a,b,c,d,M[12],6,1700485571); d=ii(d,a,b,c,M[3],10,-1894986606); c=ii(c,d,a,b,M[10],15,-1051523);  b=ii(b,c,d,a,M[1],21,-2054922799);
  a=ii(a,b,c,d,M[8],6,1873313359);  d=ii(d,a,b,c,M[15],10,-30611744);  c=ii(c,d,a,b,M[6],15,-1560198380);b=ii(b,c,d,a,M[13],21,1309151649);
  a=ii(a,b,c,d,M[4],6,-145523070);  d=ii(d,a,b,c,M[11],10,-1120210379);c=ii(c,d,a,b,M[2],15,718787259);  b=ii(b,c,d,a,M[9],21,-343485551);
  s[0]=safeAdd(a,s[0]); s[1]=safeAdd(b,s[1]); s[2]=safeAdd(c,s[2]); s[3]=safeAdd(d,s[3]);
}

window.phraseToUid = function phraseToUid(phrase) {
  if (!phrase) return '';
  var utf8  = new TextEncoder().encode(phrase);
  var len   = utf8.length;
  var bytes = Array.from(utf8);
  bytes.push(0x80);
  while (bytes.length % 64 !== 56) bytes.push(0);
  var bits = len * 8;
  bytes.push(bits & 0xff, (bits >> 8) & 0xff, (bits >> 16) & 0xff, (bits >> 24) & 0xff, 0, 0, 0, 0);

  var st = [0x67452301, 0xefcdab89|0, 0x98badcfe|0, 0x10325476];
  for (var i = 0; i < bytes.length; i += 64) {
    var M = new Array(16);
    for (var j = 0; j < 16; j++)
      M[j] = bytes[i+j*4] | (bytes[i+j*4+1] << 8) | (bytes[i+j*4+2] << 16) | (bytes[i+j*4+3] << 24);
    md5block(st, M);
  }

  var uid = [];
  for (var i = 0; i < 6; i++)
    uid.push((st[i >> 2] >>> ((i & 3) * 8)) & 0xff);
  return uid.map(function(b){ return b.toString(16).padStart(2,'0').toUpperCase(); }).join(':');
};

})();
