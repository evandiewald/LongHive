const https = require("https");
const fs = require('fs');
const {Pool, Client} = require('pg');
const { time } = require("console");


const pool = new Pool({
  user: '<username>;,
  host: 'localhost',
  database: '<database name>',
  password: '<password>',
  port: <port number>,
})

const client = new Client({
  user: '<username>',
  host: 'localhost',
  database: '<database name>',
  password: '<password>',
  port: <port>,
}) 

function getTime(unixTimestamp) {
  dateObj = new Date(unixTimestamp * 1001)
  return dateObj
}

const options = {
  hostname: "api.pipedream.com",
  port: 443,
  path: "/v1/sources/<source ID>/event_summaries?expand=event&limit=1",
  headers: {
    "Authorization": "Bearer <Bearer ID>",
  },
}

const sql_query = 'INSERT INTO hivedata_table(datetime, temperature, eC02, TVOC, queen, weight) VALUES($1, $2, $3, $4, $5, $6)'

let getData = new Promise(function(resolve, reject) {
  setTimeout(() => {
  
  const req = https.request(options,  resp => {
    let data = ""
    resp.on("data", chunk => {
      data += chunk
    })
    resp.on("end", () => {
      var obj = JSON.parse(data)
      var values = []
      var time1 = getTime(obj.data[0].event.body.hotspots[0].reported_at)
      values.push(time1)
      for (var i = 0; i<5; i++){
        values.push(obj.data[0].event.body.decoded.payload[i])
      }
        client.connect()
        // callback
        client.query(sql_query, values, (err, res) => {
          if (err) {
            console.log(err.stack)
          } else {
            console.log(res.rows[0])
          }
        })
    })
  }).on("error", err => {
    console.error("[error] " + err.message)
  })
  req.end()

  }, 2000)
})







