from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime

# Config
url = "http://localhost:8086"
token = "b_Q3pn_GmOObTb_3UZaPZc5-Scwd136XHPigfZYaa3Ou4EvhPlvIaA1nrJqzBdEBW9jDE5wrIYxZnPNydxfDzw=="
org = "camera"
bucket = "camera_optronique"

client = InfluxDBClient(url=url, token=token, org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)

point = (
    Point("temperature")
    .tag("lieu", "atelier")
    .field("value", 23.5)
    .time(datetime.utcnow(), WritePrecision.NS)
)

write_api.write(bucket=bucket, org=org, record=point)
print("✅ Donnée écrite avec succès.")
