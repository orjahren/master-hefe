
echo "Thor:"
curl http://localhost:5000/health
if [ $? -ne 0 ]; then
  exit 1
fi
echo  ""

echo "Odin:"
curl http://localhost:4000/health
if [ $? -ne 0 ]; then
  exit 1
fi
echo  ""

echo "Hefe is operational."
exit 0