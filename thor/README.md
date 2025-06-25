# Rest API - running test cases - Thor

- “Fast API”, Python
- POST a test case to the API. It will be ran on the ‘server’
- RabbitMQ for listening for finished test cases? So that the client knows it can fetch the results?
- Will need UUID for test cases so the correct result can be fetched after it has been ran
- Need to store these somewhere. NoSQL database?
- This component should also accumulate results.
- Huge TODO: What metric are these results?
- Should be containerised (Docker/Podman)
