# Landing Pages — Interest Validation

8 landing pages for testing which micro-SaaS niche gets traction.
Each page captures email signups via Formspree and tags them by niche.

## Pages

| Folder | Niche | Brand |
|---|---|---|
| `food-truck/` | Food truck permit tracking | TrackMyPermits |
| `nonprofit-grants/` | Grant management for small nonprofits | GrantTrack |
| `machine-shop/` | Maintenance logs for machine shops | ShopLog |
| `gun-shop/` | Digital bound book for FFL dealers | ComplianceLog |
| `cleaning-business/` | Ops tool for cleaning companies | CleanOps |
| `ada-compliance/` | WCAG monitoring for web agencies | SiteCheck |
| `auto-detailing/` | CRM for mobile detailers | DetailBook |
| `martial-arts/` | Dojo management for small schools | DojoLog |

---

## Setup (one-time, ~10 minutes)

### 1. Deploy to Netlify
Drag the `landing-pages/` folder to [netlify.com/drop](https://app.netlify.com/drop).
Each niche is live at `/food-truck`, `/nonprofit-grants`, etc.

### 2. Set up Formspree
1. Go to [formspree.io](https://formspree.io) → sign up → New Form
2. Set the notification email to your address
3. Copy the Form ID (looks like `xpwzabcd`)
4. Run this from inside `landing-pages/`:
   ```
   find . -name "*.html" -exec sed -i 's/FORMSPREE_ID/YOUR_ACTUAL_ID/g' {} \;
   ```
5. Redeploy to Netlify (drag again or connect git)

### 3. Check interest
```bash
cd outreach
python check_interest.py
```

Signups appear in your email (Formspree notification) and in the dashboard at formspree.io.

---

## Running the Reddit outreach

See `outreach/reddit/README.md` for Reddit setup instructions.
