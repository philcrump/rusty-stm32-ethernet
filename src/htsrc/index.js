'use strict';

let doc_system_data = null;
let doc_jsonstate_ui = null;
let doc_disconnection_ui = null;

let data_updated = 0;
let data_state = null;

function data_state_lookup(key)
{
  let result = data_state.find((element) => element[0] === key)
  return result === undefined ? undefined : result[1];
}

let view_system_data = {
  view: function()
  {
    if(data_state === null)
    {
      return;
    }

    return [
      m("p", [
        m("label", 'Firmware Uptime:'),
        m("span", `${secondsToDhms(data_state_lookup("uptime_s"))}`)
      ]),
      m("p", [
        m("label", 'MCU Temperature:'),
        m("span", `${data_state_lookup("temp_c")}°C`)
      ]),
      m("p", [
        m("label", 'Button State:'),
        m("span", `${data_state_lookup("button_state")}`)
      ])
    ];
  }
}

let view_jsonstate_ui = {
  view: function()
  {
    if(data_state === null)
    {
      return;
    }

    return m("div", {id: 'jsonstate-box'}, [
        m.trust(JsonSyntaxHighlight(JSON.stringify(data_state, undefined, 4))),
      ]
    );
  }
}


function state_update()
{
  m.request({
    method: "GET",
    url: "/api/state",
    timeout: 2000
  })
  .then((result) => {
    data_state = result;
    data_updated = Date.now();
  })
  .finally(() => {
    setTimeout(state_update, 1000);
  });
  /* Manual redraw for 'updated' */
  m.redraw();
}

window.onload = function()
{
  doc_system_data = document.getElementById('system-data');
  doc_jsonstate_ui = document.getElementById('jsonstate-ui');

  m.mount(doc_system_data, view_system_data);
  m.mount(doc_jsonstate_ui, view_jsonstate_ui);
  
  doc_disconnection_ui = document.getElementById('disconnection-ui');

  document.getElementById('toggle-rawstate').onclick = function()
  {
    if (doc_jsonstate_ui.style.display != "block")
    {
      doc_jsonstate_ui.style.display = "block";
      document.getElementById('toggle-rawstate').textContent = 'hide';
    } else {
      doc_jsonstate_ui.style.display = "none";
      document.getElementById('toggle-rawstate').textContent = 'show';
    }
  };

  state_update();

  setInterval(() => {
    m.redraw();
    if(data_updated < ((new Date()).getTime() - 3000))
    {
      doc_disconnection_ui.style.display = "block";
    }
    else
    {
      if(doc_disconnection_ui.style.display != "none")
      {
        doc_disconnection_ui.style.display = "none";
      }
    }
  }, 250);
}



function decimalFix(num, decimals, leading=0, leadingChar='0')
{
  const t = Math.pow(10, decimals);
  return ((Math.round((num * t) + (decimals>0?1:0)*(Math.sign(num) * (10 / Math.pow(100, decimals)))) / t).toFixed(decimals)).toString().padStart(leading+1+decimals, leadingChar);
}

function secondsToDhms(seconds)
{
  const d = Math.floor(seconds / (3600*24));
  const h = Math.floor(seconds % (3600*24) / 3600);
  const m = Math.floor(seconds % 3600 / 60);
  const s = Math.floor(seconds % 60);

  return  (d > 0 ? d + "d, " : "")
        + ((d+h) > 0 ? h + "h " : "")
        + ((d+h+m) > 0 ? m + "m " : "")
        + (s + "s");
}

function JsonSyntaxHighlight(json_string)
{
  json_string = json_string.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  return json_string.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, function (match) {
    let cls = 'number';
    if (/^"/.test(match)) {
      if (/:$/.test(match)) {
        cls = 'key';
      } else {
        cls = 'string';
      }
    } else if (/true|false/.test(match)) {
      cls = 'boolean';
    } else if (/null/.test(match)) {
      cls = 'null';
    }
    return '<span class="' + cls + '">' + match + '</span>';
  });
}
