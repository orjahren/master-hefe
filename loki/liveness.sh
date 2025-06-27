curl http://localhost:5000/health
if [ $? -ne 0 ]; then
  echo "Thor is not healthy"
  exit 1
else
  echo "Thor is healthy"
fi

curl http://localhost:4000/health
if [ $? -ne 0 ]; then
  echo "Odin is not healthy"
  exit 1
else
  echo "Odin is healthy"
fi

echo "Hefe is operational."
exit 0