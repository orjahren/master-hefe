
thor=$(curl -s http://localhost:5000/health | jq '.status')
if [ $? -ne 0 ]; then
  exit 1
fi
if [ "$thor" != "\"healthy\"" ]; then
  echo "Thor is not operational."
  exit 1
fi

echo "Thor: OK"

odin=$(curl -s http://localhost:4000/health | jq '.status')
if [ $? -ne 0 ]; then
  exit 1
fi
if [ "$odin" != "\"healthy\"" ]; then
  echo "Odin is not operational."
  exit 1
fi
echo "Odin: OK"

echo "Hefe is operational."
exit 0